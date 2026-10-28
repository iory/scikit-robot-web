// Runs the student's Python: Pyodide with scikit-robot, and MuJoCo's
// WebAssembly build behind a `mujoco` stand-in (mujoco_wasm.py).
//
// The program runs synchronously in this worker, so ri.wait_interpolation()
// can block the way it does on a desktop; the page stays responsive because
// it is a different thread. Frames reach it by postMessage, which a busy
// worker can still send.

import { loadPyodide } from 'https://cdn.jsdelivr.net/pyodide/v314.0.7/full/pyodide.mjs';
import loadMujoco from 'https://cdn.jsdelivr.net/npm/@mujoco/mujoco@3.17.0/mujoco.js';
import { describeModel, createCameraRenderer } from './mujoco_three.js';

// Python packages the scikit-robot wheel is installed against (it goes in
// with deps=False: its own list names desktop-only ones such as pyglet).
// Others a program imports, such as OpenCV (cv2), load when it runs.
const PYODIDE_PACKAGES = ['micropip', 'numpy', 'scipy', 'lxml', 'pillow',
  'pyyaml', 'packaging'];
// The rest are pure-Python wheels tools/build.py put in dist/wheels (listed in
// dist/wheel.json), so nothing is resolved against PyPI here. networkx is
// among them because Pyodide's own build of it pulls in matplotlib.
const CACHE_DIR = '/home/pyodide/.skrobot';

const post = (msg, transfer) => self.postMessage(msg, transfer || []);
// A key the page translates (main.js TEXT), with its arguments.
const status = (key, ...args) => post({ type: 'status', key, args });

let mujoco = null;
let pyodide = null;
const models = new WeakMap();  // MjModel -> { id, desc }
let nextModelId = 0;

function modelEntry(model) {
  let entry = models.get(model);
  if (entry === undefined) {
    entry = { id: nextModelId++, desc: describeModel(model) };
    models.set(model, entry);
    post({ type: 'model', id: entry.id, desc: entry.desc });
  }
  return entry;
}

// What mujoco_wasm.install() expects; see its docstring.
const host = {
  get mujoco() { return mujoco; },
  publish_frame(model, data) {
    const { id } = modelEntry(model);
    const xpos = data.geom_xpos.slice(), xmat = data.geom_xmat.slice();
    post({ type: 'frame', id, time: data.time, xpos, xmat },
         [xpos.buffer, xmat.buffer]);
  },
  publish_ghost(model, data, geomIds) {
    const { id } = modelEntry(model);
    const xpos = data.geom_xpos.slice(), xmat = data.geom_xmat.slice();
    post({ type: 'ghost', id, geomIds: Array.from(geomIds), xpos, xmat },
         [xpos.buffer, xmat.buffer]);
  },
  create_renderer(model, height, width) {
    const { desc } = modelEntry(model);
    const renderer = createCameraRenderer(mujoco, model, desc, height, width);
    return {
      render_rgb(data, camera) {
        const rgb = renderer.render_rgb(data, camera);
        // Show the students what their robot sees.
        post({ type: 'camera', camera, width, height, rgb: rgb.slice() });
        return rgb;
      },
      render_depth: (data, camera) => renderer.render_depth(data, camera),
    };
  },
};

// The files dist/skrobot_cache/manifest.json lists: the light Jedy's robot
// data, and the convex decompositions computed by build_cache.py.
async function fetchCache() {
  const base = new URL('./dist/skrobot_cache/', location);
  const response = await fetch(new URL('manifest.json', base));
  if (!response.ok) {
    throw new Error('dist/skrobot_cache is missing: run tools/build.py first');
  }
  const { files } = await response.json();
  return Promise.all(files.map(async (file) => {
    const r = await fetch(new URL(file, base));
    if (!r.ok) throw new Error(`${file}: HTTP ${r.status}`);
    return [file, new Uint8Array(await r.arrayBuffer())];
  }));
}

async function init() {
  // Everything that does not need Python starts at once.
  status('loadingPython');
  const mujocoReady = loadMujoco().then((m) => { mujoco = m; });
  const cacheReady = fetchCache();
  const shimReady = fetch('./mujoco_wasm.py').then((r) => {
    if (!r.ok) throw new Error(`mujoco_wasm.py: HTTP ${r.status}`);
    return r.text();
  });
  const wheelReady = fetch('./dist/wheel.json').then((r) => {
    if (!r.ok) throw new Error('dist/wheel.json is missing: run tools/build.py first');
    return r.json();
  });
  // Package-loading chatter stays in the console; the output pane shows
  // only what the student's program prints (stdout and stderr are set at
  // the end).
  pyodide = await loadPyodide();
  status('loadingPackages');
  await pyodide.loadPackage(PYODIDE_PACKAGES);

  const { wheel, deps } = await wheelReady;
  const urls = [...deps, wheel].map((file) => new URL('./dist/' + file, location).href);
  pyodide.globals.set('WHEEL_URLS', urls);
  status('installing');
  await pyodide.runPythonAsync(`
import micropip
await micropip.install(WHEEL_URLS.to_py(), deps=False)
`);

  status('loadingShapes');
  for (const [file, bytes] of await cacheReady) {
    const path = CACHE_DIR + '/' + file;
    pyodide.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
    pyodide.FS.writeFile(path, bytes);
  }
  await mujocoReady;
  // Next to the installed packages, so `import mujoco_wasm` finds it.
  const sitePackages = pyodide.runPython(
    "import sysconfig; sysconfig.get_paths()['purelib']");
  pyodide.FS.writeFile(sitePackages + '/mujoco_wasm.py', await shimReady);

  pyodide.globals.set('host', host);
  pyodide.globals.set('CACHE_DIR', CACHE_DIR);
  pyodide.runPython(`
import os
os.environ['SKROBOT_CACHE_DIR'] = CACHE_DIR
import mujoco_wasm
mujoco_wasm.install(host)
del host
`);
  pyodide.setStdout({ batched: (text) => post({ type: 'stdout', text }) });
  pyodide.setStderr({ batched: (text) => post({ type: 'stderr', text }) });
  post({ type: 'ready' });
}

// What every program imports first, done while the student reads the code;
// a run waits for it.
async function warmUp() {
  await pyodide.runPythonAsync(`
import skrobot
import skrobot.interfaces._mujoco
`);
}

const ready = init().then(warmUp).catch((e) => {
  post({ type: 'fatal', text: String(e && e.stack || e) });
  throw e;
});

self.onmessage = async (event) => {
  const msg = event.data;
  if (msg.type !== 'run') return;
  await ready;
  // Each run gets fresh globals, as a new `python script.py` would.
  const globals = pyodide.toPy({ __name__: '__main__' });
  try {
    // Packages the program imports beyond those loaded at start, such as cv2.
    // Its messages ("Loading opencv-python") would land in the output pane.
    await pyodide.loadPackagesFromImports(msg.code, { messageCallback: () => {} });
    await pyodide.runPythonAsync(msg.code, { globals, filename: 'program.py' });
    post({ type: 'done' });
  } catch (e) {
    post({ type: 'error', text: String(e.message || e) });
  } finally {
    // Draw what the program changed after the simulation last stepped,
    // such as robot.angle_vector(ri.angle_vector()) on its last line.
    try {
      pyodide.runPython('import mujoco.viewer; mujoco.viewer.finish()');
    } catch (e) {
      post({ type: 'error', text: String(e.message || e) });
    }
    globals.destroy();
    // ri refers to itself (its frame callbacks), so only the cycle
    // collector frees it, and with it the MuJoCo model it holds.
    pyodide.runPython('import gc; gc.collect()');
  }
};
