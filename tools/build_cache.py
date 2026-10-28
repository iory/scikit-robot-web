"""Build the skrobot cache the page starts with, under ``site/dist/skrobot_cache``.

- **Robot data.** The samples load ``Jedy(light=True)``, whose meshes are
  simplified (about 110,000 faces instead of 700,000, which take the
  browser tens of seconds to load). Its data is fetched here once and
  served with the page.
- **Convex decomposition.** CoACD is a native library, so Pyodide has none.
  The decompositions the samples ask for are computed here, and the page
  finds them in skrobot's cache (``convex_decomposition_cached``).

``manifest.json`` lists the files for the page to fetch. The decomposition
cache is keyed by mesh bytes and CoACD settings, so the converter kwargs in
``ROBOTS`` must match what the samples pass as ``converter_kwargs``, and the
page must load meshes with the same trimesh (``build.py`` records its
version).

    python tools/build_cache.py site/dist/skrobot_cache   # build.py runs it
"""

import json
import os
import shutil
import sys
import tempfile


# Robot class name -> (robot data directory, constructor kwargs, and
# converter_kwargs as the samples pass them).
ROBOTS = {
    'Jedy': ('jedy_light_description', {'light': True}, {
        'convex_decompose_links': ['rarm_link7', 'rarm_link8',
                                   'larm_link7', 'larm_link8'],
        'coacd_quality': 'fine',
    }),
}


def main():
    out_dir = os.path.abspath(sys.argv[1])
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    # Set before skrobot reads it: robot data downloads here, and the
    # decompositions land in <out_dir>/convex_decomposition.
    os.environ['SKROBOT_CACHE_DIR'] = out_dir

    import skrobot
    from skrobot.urdf import urdf_to_mjcf

    for robot_name, (data_dir, robot_kwargs, kwargs) in ROBOTS.items():
        # Downloads the robot data into out_dir.
        robot = getattr(skrobot.models, robot_name)(**robot_kwargs)
        work = tempfile.mkdtemp()
        try:
            urdf_to_mjcf(robot.urdf_path, os.path.join(work, 'robot.xml'),
                         **kwargs)
        finally:
            shutil.rmtree(work)

    keep = {'convex_decomposition'} | {d for d, _, _ in ROBOTS.values()}
    for name in os.listdir(out_dir):
        if name not in keep:
            path = os.path.join(out_dir, name)
            if os.path.isdir(path):
                shutil.rmtree(path)
            else:
                os.remove(path)
    files = []
    for dirpath, _, filenames in os.walk(out_dir):
        for filename in filenames:
            files.append(os.path.relpath(os.path.join(dirpath, filename),
                                         out_dir))
    with open(os.path.join(out_dir, 'manifest.json'), 'w') as f:
        json.dump({'files': sorted(files)}, f, indent=1)
    print('cached {} files in {}'.format(len(files), out_dir))


if __name__ == '__main__':
    main()
