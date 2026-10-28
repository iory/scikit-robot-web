// Draw a MuJoCo model with three.js.
//
// Used twice: by the page, for the 3D view, from a description the worker
// posts; and inside the worker, for the robot's cameras (ri.get_image /
// ri.get_depth), on an OffscreenCanvas.

import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js';

export { THREE };

// mjtGeom
const PLANE = 0, SPHERE = 2, CAPSULE = 3, ELLIPSOID = 4, CYLINDER = 5,
  BOX = 6, MESH = 7;

// Geom groups MuJoCo's own viewer shows by default. skrobot's converter
// puts visual geoms in 2 and collision geoms in 3.
const VISIBLE_GROUPS = new Set([0, 1, 2]);

const PLANE_HALF_SIZE = 20.0;  // for an infinite plane (size 0)

const GHOST_COLOR = new THREE.Color(0.2, 0.55, 1.0);

// Copies of the model arrays the drawing needs, safe to postMessage.
export function describeModel(model) {
  const copy = (name) => model[name].slice();
  return {
    ngeom: model.ngeom,
    geom_type: copy('geom_type'),
    geom_size: copy('geom_size'),
    geom_rgba: copy('geom_rgba'),
    geom_group: copy('geom_group'),
    geom_dataid: copy('geom_dataid'),
    geom_matid: copy('geom_matid'),
    mat_rgba: copy('mat_rgba'),
    mesh_vertadr: copy('mesh_vertadr'),
    mesh_vertnum: copy('mesh_vertnum'),
    mesh_faceadr: copy('mesh_faceadr'),
    mesh_facenum: copy('mesh_facenum'),
    mesh_vert: copy('mesh_vert'),
    mesh_face: copy('mesh_face'),
    extent: model.stat.extent,
    zfar: model.vis.map.zfar,
    znear: model.vis.map.znear,
  };
}

function meshGeometry(desc, meshId) {
  const v0 = desc.mesh_vertadr[meshId], nv = desc.mesh_vertnum[meshId];
  const f0 = desc.mesh_faceadr[meshId], nf = desc.mesh_facenum[meshId];
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(
    desc.mesh_vert.slice(3 * v0, 3 * (v0 + nv)), 3));
  // MuJoCo stores faces as Int32; WebGL indices must be unsigned.
  geometry.setIndex(new THREE.BufferAttribute(
    Uint32Array.from(desc.mesh_face.subarray(3 * f0, 3 * (f0 + nf))), 1));
  geometry.computeVertexNormals();
  return geometry;
}

function geomGeometry(desc, i) {
  const s = desc.geom_size.subarray(3 * i, 3 * i + 3);
  switch (desc.geom_type[i]) {
    case PLANE:
      return new THREE.PlaneGeometry(
        2 * (s[0] || PLANE_HALF_SIZE), 2 * (s[1] || PLANE_HALF_SIZE));
    case SPHERE:
      return new THREE.SphereGeometry(s[0], 24, 16);
    case ELLIPSOID: {
      const g = new THREE.SphereGeometry(1, 24, 16);
      g.scale(s[0], s[1], s[2]);
      return g;
    }
    case CAPSULE: {
      // three.js builds it along y; MuJoCo's runs along z.
      const g = new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 16);
      g.rotateX(Math.PI / 2);
      return g;
    }
    case CYLINDER: {
      const g = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24);
      g.rotateX(Math.PI / 2);
      return g;
    }
    case BOX:
      return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    case MESH:
      return meshGeometry(desc, desc.geom_dataid[i]);
    default:
      return null;  // heightfields and the rest are not drawn
  }
}

function geomColor(desc, i) {
  const mat = desc.geom_matid[i];
  const rgba = mat >= 0
    ? desc.mat_rgba.subarray(4 * mat, 4 * mat + 4)
    : desc.geom_rgba.subarray(4 * i, 4 * i + 4);
  return rgba;
}

// A THREE.Group with one object per drawn geom; posed by setPoses().
// With `only` (geom ids), just those; with `ghost`, all of one translucent
// colour, drawn over the rest.
export function buildModel(desc, { only = null, ghost = false } = {}) {
  const root = new THREE.Group();
  const objects = new Array(desc.ngeom).fill(null);
  const wanted = only === null ? null : new Set(only);
  for (let i = 0; i < desc.ngeom; i++) {
    if (!VISIBLE_GROUPS.has(desc.geom_group[i])) continue;
    if (wanted !== null && !wanted.has(i)) continue;
    const geometry = geomGeometry(desc, i);
    if (geometry === null) continue;
    const rgba = geomColor(desc, i);
    if (rgba[3] === 0) continue;
    const material = ghost
      ? new THREE.MeshStandardMaterial({
        color: GHOST_COLOR, transparent: true, opacity: 0.35,
        depthWrite: false, roughness: 0.6,
      })
      : new THREE.MeshStandardMaterial({
        color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
        // A see-through geom must not hide what is inside it (the ghost).
        transparent: rgba[3] < 1, opacity: rgba[3], depthWrite: rgba[3] >= 1,
        roughness: 0.7, metalness: 0.1,
        side: desc.geom_type[i] === PLANE ? THREE.DoubleSide : THREE.FrontSide,
      });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.matrixAutoUpdate = false;
    mesh.userData.isPlane = desc.geom_type[i] === PLANE;
    if (ghost) mesh.renderOrder = 1;  // after the opaque robot it overlaps
    root.add(mesh);
    objects[i] = mesh;
  }
  return { root, objects };
}

// geom_xpos (ngeom x 3) and geom_xmat (ngeom x 9, row-major) onto the
// objects buildModel() made.
export function setPoses(objects, xpos, xmat) {
  for (let i = 0; i < objects.length; i++) {
    const o = objects[i];
    if (o === null) continue;
    const p = 3 * i, r = 9 * i;
    o.matrix.set(
      xmat[r], xmat[r + 1], xmat[r + 2], xpos[p],
      xmat[r + 3], xmat[r + 4], xmat[r + 5], xpos[p + 1],
      xmat[r + 6], xmat[r + 7], xmat[r + 8], xpos[p + 2],
      0, 0, 0, 1);
    o.matrixWorldNeedsUpdate = true;
  }
}

export function addLights(scene) {
  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(2, -3, 5);
  scene.add(sun);
}

const DEPTH_MATERIAL = new THREE.ShaderMaterial({
  vertexShader: `
    varying float vDepth;
    void main() {
      vec4 mv = modelViewMatrix * vec4(position, 1.0);
      vDepth = -mv.z;
      gl_Position = projectionMatrix * mv;
    }`,
  fragmentShader: `
    varying float vDepth;
    void main() { gl_FragColor = vec4(vDepth, 0.0, 0.0, 1.0); }`,
  side: THREE.DoubleSide,
});

// mujoco.Renderer for one image size, for the worker. render_rgb returns
// height*width*3 bytes and render_depth height*width metres along the
// optical axis, both top row first, as the Python mujoco.Renderer does.
// Where nothing is hit, depth is the far clipping distance, as in MuJoCo.
export function createCameraRenderer(mujoco, model, desc, height, width) {
  const canvas = new OffscreenCanvas(width, height);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false });
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0.55, 0.65, 0.75);
  addLights(scene);
  const { root, objects } = buildModel(desc);
  scene.add(root);
  const near = desc.znear * desc.extent, far = desc.zfar * desc.extent;
  const camera = new THREE.PerspectiveCamera(45, width / height, near, far);
  camera.matrixAutoUpdate = false;

  const colorTarget = new THREE.WebGLRenderTarget(width, height);
  colorTarget.texture.colorSpace = THREE.SRGBColorSpace;
  const depthTarget = new THREE.WebGLRenderTarget(width, height, {
    type: THREE.FloatType });
  const rgba = new Uint8Array(width * height * 4);
  const depthRGBA = new Float32Array(width * height * 4);

  function prepare(data, cameraName) {
    const cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA.value,
                                  cameraName);
    if (cid < 0) throw new Error('no camera ' + cameraName);
    setPoses(objects, data.geom_xpos, data.geom_xmat);
    // A MuJoCo camera looks along -z with +y up, as a three.js one does.
    const p = data.cam_xpos.subarray(3 * cid, 3 * cid + 3);
    const r = data.cam_xmat.subarray(9 * cid, 9 * cid + 9);
    camera.fov = model.cam_fovy[cid];
    camera.updateProjectionMatrix();
    camera.matrix.set(r[0], r[1], r[2], p[0], r[3], r[4], r[5], p[1],
                      r[6], r[7], r[8], p[2], 0, 0, 0, 1);
    camera.matrixWorld.copy(camera.matrix);
    camera.matrixWorldInverse.copy(camera.matrix).invert();
  }

  return {
    render_rgb(data, cameraName) {
      prepare(data, cameraName);
      renderer.setRenderTarget(colorTarget);
      renderer.render(scene, camera);
      renderer.readRenderTargetPixels(colorTarget, 0, 0, width, height, rgba);
      renderer.setRenderTarget(null);
      const out = new Uint8Array(width * height * 3);
      for (let y = 0; y < height; y++) {
        const src = (height - 1 - y) * width * 4, dst = y * width * 3;
        for (let x = 0; x < width; x++) {
          out[dst + 3 * x] = rgba[src + 4 * x];
          out[dst + 3 * x + 1] = rgba[src + 4 * x + 1];
          out[dst + 3 * x + 2] = rgba[src + 4 * x + 2];
        }
      }
      return out;
    },
    render_depth(data, cameraName) {
      prepare(data, cameraName);
      const background = scene.background;
      scene.background = null;
      scene.overrideMaterial = DEPTH_MATERIAL;
      renderer.setRenderTarget(depthTarget);
      renderer.setClearColor(new THREE.Color(far, 0, 0), 1);
      renderer.clear();
      renderer.render(scene, camera);
      renderer.readRenderTargetPixels(depthTarget, 0, 0, width, height,
                                      depthRGBA);
      renderer.setRenderTarget(null);
      renderer.setClearColor(new THREE.Color(0, 0, 0), 1);
      scene.overrideMaterial = null;
      scene.background = background;
      const out = new Float32Array(width * height);
      for (let y = 0; y < height; y++) {
        const src = (height - 1 - y) * width * 4, dst = y * width;
        for (let x = 0; x < width; x++) out[dst + x] = depthRGBA[src + 4 * x];
      }
      return out;
    },
  };
}
