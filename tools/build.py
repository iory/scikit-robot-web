"""Build what the page installs into Pyodide, under ``site/dist/``.

- a scikit-robot wheel of the scikit-robot installed where this runs, and
  the pure-Python wheels it needs beyond Pyodide's own packages, so the
  page installs everything from here without asking PyPI;
- the robot data the samples load and the convex decompositions only a
  desktop can compute (build_cache.py), made with that same scikit-robot;
- ``wheel.json``, naming the wheel and the trimesh version the
  decompositions were computed with: the page must load meshes with the
  same trimesh, or the cached decompositions (keyed by mesh bytes) would
  not be found.

Needs pip, coacd and scikit-robot in the running environment::

    uv run --with pip --with coacd --with scikit-robot python tools/build.py
    # or a scikit-robot checkout, for one not released yet:
    uv run --with pip --with coacd --with ../scikit-robot python tools/build.py
    python -m http.server -d site 8000   # open http://localhost:8000
"""

import glob
import importlib.metadata
import json
import os
import shutil
import subprocess
import sys
import urllib.parse


HERE = os.path.dirname(os.path.abspath(__file__))
# What is served: the page, and dist/ in it.
SITE = os.path.join(os.path.dirname(HERE), 'site')
# What scikit-robot needs in Pyodide besides the packages worker.js loads
# from Pyodide itself (PYODIDE_PACKAGES), with their own dependencies.
# Pinned to the versions installed here when there are any: trimesh must be,
# since the decompositions are keyed by the mesh bytes it loads.
# Pyodide's Python (worker.js loads Pyodide 314.0.7).
PYODIDE_PYTHON = '3.14'
RUNTIME_WHEELS = ['cached-property', 'ordered-set', 'filelock', 'networkx',
                  'trimesh', 'pycollada', 'python-dateutil', 'six', 'pooch',
                  'platformdirs', 'requests', 'urllib3', 'certifi', 'idna',
                  'charset-normalizer']


def download_runtime_wheels(out_dir):
    requirements = []
    for name in RUNTIME_WHEELS:
        try:
            requirements.append('{}=={}'.format(
                name, importlib.metadata.version(name)))
        except importlib.metadata.PackageNotFoundError:
            requirements.append(name)
    # Pure-Python wheels only: they are what Pyodide can install.
    subprocess.check_call([
        sys.executable, '-m', 'pip', 'download', '--no-deps',
        '--only-binary=:all:', '--platform', 'any', '--implementation', 'py',
        '--abi', 'none', '--python-version', PYODIDE_PYTHON, '--dest', out_dir]
        + requirements)
    return sorted(os.path.basename(path)
                  for path in glob.glob(os.path.join(out_dir, '*.whl')))


def build_skrobot_wheel(out_dir):
    """Put a wheel of the installed scikit-robot in ``out_dir``: built from
    the directory or git commit it was installed from, else downloaded."""
    dist = importlib.metadata.distribution('scikit-robot')
    direct_url = json.loads(dist.read_text('direct_url.json') or '{}')
    url = direct_url.get('url', '')
    vcs = direct_url.get('vcs_info')
    if url.startswith('file://'):
        source = urllib.parse.unquote(urllib.parse.urlparse(url).path)
    elif vcs:
        source = '{}+{}@{}'.format(vcs['vcs'], url, vcs['commit_id'])
    else:
        source = None
    if source:
        subprocess.check_call([sys.executable, '-m', 'pip', 'wheel',
                               '--no-deps', '--wheel-dir', out_dir, source])
    else:
        subprocess.check_call([
            sys.executable, '-m', 'pip', 'download', '--no-deps',
            '--only-binary=:all:', '--dest', out_dir,
            'scikit-robot=={}'.format(dist.version)])


def main():
    out_dir = os.path.join(SITE, 'dist')
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    build_skrobot_wheel(out_dir)
    wheels = glob.glob(os.path.join(out_dir, 'scikit_robot-*.whl'))
    if len(wheels) != 1:
        raise RuntimeError('expected one scikit-robot wheel, found {}'.format(
            wheels))
    # A process of its own: it points skrobot's cache at the output before
    # skrobot is imported.
    subprocess.check_call([sys.executable, os.path.join(HERE, 'build_cache.py'),
                           os.path.join(out_dir, 'skrobot_cache')])
    deps = download_runtime_wheels(os.path.join(out_dir, 'wheels'))
    info = {'wheel': os.path.basename(wheels[0]),
            'deps': ['wheels/' + name for name in deps],
            'trimesh': importlib.metadata.version('trimesh')}
    with open(os.path.join(out_dir, 'wheel.json'), 'w') as f:
        json.dump(info, f, indent=1)
    print('built {} with {} wheels (trimesh {})'.format(
        wheels[0], len(deps), info['trimesh']))


if __name__ == '__main__':
    main()
