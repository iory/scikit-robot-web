"""The ``mujoco`` Python API over MuJoCo's WebAssembly build, for Pyodide.

In a browser, Python runs in Pyodide and MuJoCo in its own WebAssembly
module (``@mujoco/mujoco``), each with its own memory and file system. This
module stands in for the ``mujoco`` package on that side: ``install()``
registers it as ``mujoco`` and ``mujoco.viewer``, after which
scikit-robot's ``MujocoRobotInterface`` -- and code written
against it on a desktop -- runs unchanged.

It covers what the interface uses, not the whole API. Array fields come back
as ``_Field``, which reads a copy out of MuJoCo's memory on every index
and writes assignments back, because Pyodide cannot hold a NumPy view onto
another module's memory.

The page provides a ``host`` JavaScript object:

``host.mujoco``
    The loaded ``@mujoco/mujoco`` module.
``host.publish_frame(model, data)``
    Called by ``viewer.sync()``; draws the current state.
``host.publish_ghost(model, data, geom_ids)``
    Called by ``viewer.sync_ghost()``; draws those geoms of ``data`` (the
    robot model's posture) translucent.
``host.create_renderer(model, height, width)``
    Returns an object with ``render_rgb(data, camera)`` (a ``Uint8Array``
    of ``height * width * 3``, top row first) and
    ``render_depth(data, camera)`` (a ``Float32Array`` of metres).
"""

import os
import sys
import types
import weakref

import numpy as np


_DTYPES = {
    'Float64Array': np.float64,
    'Float32Array': np.float32,
    'Int32Array': np.int32,
    'Uint32Array': np.uint32,
    'Int8Array': np.int8,
    'Uint8Array': np.uint8,
    'Int16Array': np.int16,
    'Uint16Array': np.uint16,
}

# Leading dimension of a field, by name prefix, as the Python bindings shape
# them; the trailing one is the length divided by it.
_ROWS = (
    ('actuator_', 'nu'), ('body_', 'nbody'), ('cam_', 'ncam'),
    ('dof_', 'nv'), ('geom_', 'ngeom'), ('jnt_', 'njnt'),
    ('light_', 'nlight'), ('site_', 'nsite'),
    ('xpos', 'nbody'), ('xquat', 'nbody'), ('xmat', 'nbody'),
    ('xipos', 'nbody'), ('ximat', 'nbody'), ('subtree_', 'nbody'),
    ('cvel', 'nbody'), ('cinert', 'nbody'),
)


def _is_typed_array(value):
    return hasattr(value, 'BYTES_PER_ELEMENT')


class _Field(object):
    """A MuJoCo array, indexed and assigned like the NumPy one it mirrors.

    It re-reads the typed array on each access: growing the WebAssembly
    memory detaches views taken earlier.
    """

    def __init__(self, owner, name, rows):
        self._owner = owner
        self._name = name
        self._rows = rows

    def _js(self):
        return getattr(self._owner, self._name)

    def _read(self):
        js = self._js()
        out = np.empty(js.length, dtype=_DTYPES[js.constructor.name])
        js.assign_to(out)
        if self._rows and js.length != self._rows:
            out = out.reshape(self._rows, -1)
        return out

    def __array__(self, dtype=None, copy=None):
        out = self._read()
        return out if dtype is None else out.astype(dtype)

    def __getitem__(self, key):
        return self._read()[key]

    def __setitem__(self, key, value):
        out = self._read()
        out[key] = value
        self._js().assign(np.ascontiguousarray(out).ravel())

    def __len__(self):
        return len(self._read())

    @property
    def shape(self):
        return self._read().shape

    def copy(self):
        return self._read()

    def reshape(self, *shape):
        return self._read().reshape(*shape)


class _Contact(object):
    """One contact, copied out of ``MjData.contact``."""

    def __init__(self, js):
        self.geom1 = js.geom1
        self.geom2 = js.geom2
        self.dist = js.dist


class _Struct(object):
    """Attribute access to an MjModel, MjData or MjOption handle."""

    def __init__(self, js, sizes=None):
        object.__setattr__(self, '_js', js)
        # The object whose n* counts shape this one's fields (the model).
        object.__setattr__(self, '_sizes', sizes if sizes is not None else js)

    def __getattr__(self, name):
        if name == 'contact':
            vec = self._js.contact
            try:
                return [_Contact(vec.get(i)) for i in range(vec.size())]
            finally:
                vec.delete()
        value = getattr(self._js, name)
        if _is_typed_array(value):
            rows = None
            for prefix, count in _ROWS:
                if name.startswith(prefix):
                    rows = getattr(self._sizes, count)
                    break
            return _Field(self._js, name, rows)
        if hasattr(value, 'delete') and not callable(value):
            return _Struct(value, self._sizes)
        if type(value).__name__ == 'JsProxy' and hasattr(value, '$$'):
            return _Struct(value, self._sizes)
        return value

    def __setattr__(self, name, value):
        setattr(self._js, name, value)


class _Owned(_Struct):
    """An MjModel or MjData this module created, and so must free.

    Embind objects live on MuJoCo's heap, outside JavaScript's garbage
    collector; left alone, every program run leaks a model and the module
    runs out of memory a run or two later.
    """

    def __init__(self, js, sizes=None, keep=None):
        super(_Owned, self).__init__(js, sizes)
        # MjData's fields are shaped by its model's counts, so the model
        # must outlive it.
        object.__setattr__(self, '_keep', keep)

    def __del__(self):
        try:
            self._js.delete()
        except Exception:
            pass  # already freed, or the module is being torn down


class _Enum(object):
    """``mujoco.mjtObj`` and friends: members are plain ints."""

    def __init__(self, js_enum):
        self._js_enum = js_enum

    def __getattr__(self, name):
        return getattr(self._js_enum, name).value


def _unwrap(value):
    if isinstance(value, _Struct):
        return value._js
    # A NumPy scalar would cross as a proxy object, which Embind refuses
    # where it expects an int.
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


# path -> (mtime, size) of each file already in MuJoCo's file system.
_copied = {}


def _copy_tree_to(fs, directory):
    """Copy ``directory`` from Pyodide's file system into MuJoCo's.

    A file copied before and unchanged since is skipped: a robot's meshes
    run to hundreds of files, and a program run again loads the same ones.
    """
    from pyodide.ffi import to_js

    for dirpath, _, filenames in os.walk(directory):
        fs.mkdirTree(dirpath)
        for filename in filenames:
            path = os.path.join(dirpath, filename)
            stat = os.stat(path)
            stamp = (stat.st_mtime, stat.st_size)
            if _copied.get(path) == stamp:
                continue
            with open(path, 'rb') as f:
                fs.writeFile(path, to_js(f.read()))
            _copied[path] = stamp


def _make_module(host):
    js = host.mujoco
    mujoco = types.ModuleType('mujoco')
    mujoco.__doc__ = __doc__

    class MjModel(_Owned):
        @staticmethod
        def from_xml_path(path):
            path = os.path.abspath(path)
            # The XML's meshdir is relative, so its whole directory goes.
            _copy_tree_to(js.FS, os.path.dirname(path))
            return MjModel(js.MjModel.from_xml_path(path))

        @staticmethod
        def from_xml_string(xml):
            return MjModel(js.MjModel.from_xml_string(xml))

    def MjData(model):
        return _Owned(js.MjData.new(model._js), model._js, keep=model)

    def mj_mulM(model, data, res, vec):
        from pyodide.ffi import to_js

        buf = js.DoubleBuffer.new(len(res))
        try:
            js.mj_mulM(model._js, data._js, buf,
                       to_js([float(v) for v in vec]))
            buf.GetView().assign_to(res)
        finally:
            buf.delete()

    class Renderer(object):
        """``mujoco.Renderer``, drawn by the page."""

        def __init__(self, model, height=240, width=320):
            self._js = host.create_renderer(model._js, height, width)
            self._height = height
            self._width = width
            self._depth = False
            self._data = None
            self._camera = None

        def update_scene(self, data, camera=None):
            self._data = data._js
            self._camera = camera

        def enable_depth_rendering(self):
            self._depth = True

        def disable_depth_rendering(self):
            self._depth = False

        def render(self):
            if self._depth:
                out = np.empty(self._height * self._width, dtype=np.float32)
                self._js.render_depth(self._data, self._camera).assign_to(out)
                return out.reshape(self._height, self._width)
            out = np.empty(self._height * self._width * 3, dtype=np.uint8)
            self._js.render_rgb(self._data, self._camera).assign_to(out)
            return out.reshape(self._height, self._width, 3)

    mujoco.MjModel = MjModel
    mujoco.MjData = MjData
    mujoco.mj_mulM = mj_mulM
    mujoco.Renderer = Renderer
    for enum in ('mjtObj', 'mjtJoint', 'mjtGeom', 'mjtIntegrator'):
        setattr(mujoco, enum, _Enum(getattr(js, enum)))

    def passthrough(name):
        fn = getattr(js, name)

        def call(*args):
            return fn(*[_unwrap(a) for a in args])
        call.__name__ = name
        return call

    for name in ('mj_step', 'mj_forward', 'mj_kinematics', 'mj_resetData',
                 'mj_name2id', 'mj_id2name'):
        setattr(mujoco, name, passthrough(name))
    return mujoco


def _make_viewer(host):
    viewer = types.ModuleType('mujoco.viewer')
    # Viewers still open, for finish(). Weak, so a program's ri (and the
    # MuJoCo model it holds) is still freed once nothing else refers to it.
    open_handles = weakref.WeakSet()

    class _Handle(object):
        def __init__(self, model, data):
            self._model = model
            self._data = data
            # Called by finish(): frames are drawn only while the
            # simulation steps, so what a program changes after its last
            # step (the robot model, say) needs drawing once more.
            self.on_finish = []

        def sync(self):
            host.publish_frame(self._model._js, self._data._js)

        def sync_ghost(self, data, geom_ids):
            from pyodide.ffi import to_js

            host.publish_ghost(self._model._js, data._js, to_js(geom_ids))

        def close(self):
            pass

    def launch_passive(model, data):
        handle = _Handle(model, data)
        handle.sync()
        open_handles.add(handle)
        return handle

    def finish():
        """Draw every open viewer once more; the host calls it when a
        program ends."""
        for handle in list(open_handles):
            handle.sync()
            for callback in handle.on_finish:
                callback()

    viewer.launch_passive = launch_passive
    viewer.finish = finish
    return viewer


def install(host):
    """Register the WebAssembly-backed module as ``mujoco``.

    Parameters
    ----------
    host : pyodide.ffi.JsProxy
        See the module docstring.

    Returns
    -------
    module
        The module now importable as ``mujoco``.
    """
    mujoco = _make_module(host)
    mujoco.viewer = _make_viewer(host)
    sys.modules['mujoco'] = mujoco
    sys.modules['mujoco.viewer'] = mujoco.viewer
    return mujoco
