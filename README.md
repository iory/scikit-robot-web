# scikit-robot-web

[日本語](README.ja.md)

**Try it:** https://iory.github.io/scikit-robot-web/ (English) or
[in Japanese](https://iory.github.io/scikit-robot-web/?lang=ja).

[![Web ri: Python on the left, the simulated Jedy on the right](docs/screenshot.png)](https://iory.github.io/scikit-robot-web/)

Program a robot from a web page, with nothing to install. You write Python
in the page and it runs there: [scikit-robot](https://github.com/iory/scikit-robot)
runs in [Pyodide](https://pyodide.org/) (Python compiled to WebAssembly),
and the robot is simulated by [MuJoCo](https://mujoco.org/)'s WebAssembly
build. The robot is Jedy, JSK's small dual-arm mobile robot. The server
only hands out static files.

The same programs run on a desktop, where scikit-robot's
`MujocoRobotInterface` uses the `mujoco` Python package.

## robot and ri

Programs for a real robot in scikit-robot (and in EusLisp before it) keep
two objects apart:

- **`robot`**: the robot model you plan with: forward and inverse
  kinematics, collision checks. Changing it moves nothing. It changes at
  once, and passes through tables.
- **`ri`**: the robot interface. It sends commands to the robot and reads
  back what the robot did. Motion takes time, and gravity and contact can
  stop the robot short of the command.

The loop is: plan with `robot`, move with `ri`, check with `ri`:

```python
robot.inverse_kinematics(target, link_list=robot.rarm.link_list,
                         move_target=robot.rarm_end_coords)
ri.angle_vector(robot.angle_vector(), 2.0)  # send, reach it in 2 s
ri.wait_interpolation()                     # block until it is done
robot.angle_vector(ri.angle_vector())       # where it actually is
```

`ri` never writes into `robot`: it works on its own copy, as the ROS
interface does. In the page's 3D view, the translucent blue robot is
`robot` and the solid one is the simulated robot `ri` drives, so the
difference can be seen. Only the line that creates `ri` differs from a
program for the real robot.

## Samples

The page opens with four samples, in `site/samples/en` (and `site/samples/ja`):

1. **Basics**: send joint angles with `ri.angle_vector`, wait with
   `ri.wait_interpolation`, drive with `ri.go_pos`, read the robot back.
2. **Pick a cube**: drive to a table, reach with inverse kinematics, lift a
   cube, asking `ri` where the cube is.
3. **robot vs ri**: plan a hand inside a table; the model passes through,
   the robot stops on the top.
4. **robot vs ri in a pick**: the pick again, stopping where `robot` and
   `ri` differ: planning, driving the base, closing the gripper on the
   cube, and the cube's position.

They run on a desktop as well (`viewer=True` opens MuJoCo's viewer):

```sh
pip install scikit-robot[mujoco] coacd
python site/samples/en/02_pick.py
```

## What ri offers

| Call | Does |
| --- | --- |
| `ri.angle_vector(av, time)` | Send joint angles, reached in `time` seconds (raised to the joint velocity limits). Returns at once. |
| `ri.angle_vector_sequence(avs, times)` | Send several, passed through without stopping. |
| `ri.wait_interpolation()` | Wait until the motion sent last is over. |
| `ri.angle_vector()` | The robot's joint angles now. |
| `ri.go_pos(x, y, yaw)`, `ri.go_velocity(...)`, `ri.move_to(coords)` | Drive the base (`planar_base=True`), in its own frame for `go_pos`. |
| `ri.odom` | Where the base is. |
| `ri.object_coords(name)`, `ri.contacts(name)` | Where an object is, and what touches it. |
| `ri.get_image(camera)`, `ri.get_depth(camera)` | Camera images, from cameras given with `cameras=`. |

`ri.object_coords` and `ri.contacts` give the simulation's ground truth,
which a real robot does not have; there, they come from sensors such as
cameras.

## Building and serving

The page is static: any web server can host `site/`. `tools/build.py`
builds what the page installs into Pyodide, from the scikit-robot installed
where it runs:

```sh
uv run --with pip --with coacd --with scikit-robot python tools/build.py
# or from a scikit-robot checkout:
uv run --with pip --with coacd --with ../scikit-robot python tools/build.py
python -m http.server -d site 8000   # open http://localhost:8000
```

It writes `site/dist/`:

- a wheel of that scikit-robot and the pure-Python wheels it needs, so the
  page does not query PyPI;
- the robot data of the light Jedy the samples load (`Jedy(light=True)`:
  about 110,000 faces instead of 700,000, which take the browser tens of
  seconds to convert and compile);
- convex decompositions of Jedy's fingers. MuJoCo collides meshes by
  their convex hull, which would fill the space between the fingers, so
  those links are split into convex parts with CoACD
  (`converter_kwargs={'convex_decompose_links': [...]}`). CoACD is a
  native library that does not run in the browser, so this is done here
  and the page loads the results into scikit-robot's cache.

`.github/workflows/pages.yml` builds it and publishes `site/` on GitHub
Pages at every push to `main`.

| File | Role |
| --- | --- |
| `site/index.html`, `style.css`, `main.js` | Editor, output, 3D view; English and Japanese text. |
| `site/worker.js` | Web Worker that loads Pyodide and MuJoCo and runs the program. The program runs synchronously there, so `ri.wait_interpolation()` blocks as on a desktop, while the page stays responsive. |
| `site/mujoco_wasm.py` | Presents MuJoCo's WebAssembly build as the `mujoco` Python package, so `MujocoRobotInterface` runs unchanged. |
| `site/mujoco_three.js` | Draws MuJoCo models with three.js, for the 3D view and for camera images. |
| `site/samples/en`, `site/samples/ja` | The samples, one copy per language. |
| `tools/build.py`, `tools/build_cache.py` | Build `site/dist/`. |

The decompositions are cached by mesh bytes and CoACD settings, so `ROBOTS`
in `tools/build_cache.py` must match the samples' `converter_kwargs`, and the
page loads the trimesh version `build.py` recorded in `site/dist/wheel.json`.

## Limitations

- **Stop** restarts Python (about ten seconds): interrupting running
  Python needs cross-origin isolation headers, which static hosts such as
  GitHub Pages do not send.
- The first run converts the robot, which takes several seconds.
- Simulated time passes only inside `ri` calls (`wait_interpolation`,
  `go_pos` and the like); `time.sleep` does not move the robot, and a loop
  polling `ri.is_interpolating()` does not end.
- Camera images are drawn with three.js, so colours and shading differ
  from MuJoCo's own renderer. Depth is in metres along the optical axis.
- Tested with Chrome.
