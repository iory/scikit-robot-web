// The page: editor, output and the 3D view of what the worker simulates.

import { THREE, buildModel, setPoses, addLights } from './mujoco_three.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// --- language ------------------------------------------------------------

const TEXT = {
  en: {
    docs: 'Docs', sample: 'Sample', run: 'Run', stop: 'Stop',
    revert: 'Reset code', revertTitle: 'Discard your edits and reload the sample',
    output: 'Output', shortcut: 'Ctrl+Enter to run', clear: 'Clear',
    preparing: 'Preparing the simulation… converting the robot takes a few seconds the first time.',
    legendRobot: 'robot (model)', legendRi: 'ri (simulated robot)',
    fit: 'Reset view',
    loadingPython: 'Loading Python and MuJoCo…',
    loadingPackages: 'Loading NumPy and SciPy…',
    installing: 'Installing scikit-robot…',
    loadingShapes: 'Loading the robot…',
    ready: 'Ready', running: 'Running', done: 'Finished', error: 'Error',
    failed: 'Failed to start',
    stopped: 'Stopped. Restarting Python…',
    discard: 'Discard your edits to the code?',
  },
  ja: {
    docs: 'ドキュメント', sample: 'サンプル', run: '実行', stop: '停止',
    revert: 'コードを戻す', revertTitle: '編集を捨ててサンプルを読み直す',
    output: '出力', shortcut: 'Ctrl+Enter で実行', clear: 'クリア',
    preparing: 'シミュレーションを準備中…（初回はロボットの変換に数秒かかります）',
    legendRobot: 'robot（模型）', legendRi: 'ri（シミュレーションのロボット）',
    fit: '視点を戻す',
    loadingPython: 'Python と MuJoCo を読み込み中…',
    loadingPackages: 'NumPy と SciPy を読み込み中…',
    installing: 'scikit-robot をインストール中…',
    loadingShapes: 'ロボットのデータを読み込み中…',
    ready: '準備完了', running: '実行中', done: '終了', error: 'エラー',
    failed: '起動に失敗しました',
    stopped: '停止しました。Python を再起動しています…',
    discard: '編集したコードを捨てますか？',
  },
};

const SAMPLES = [
  {
    file: '01_basics.py',
    en: ['1. Basics: joints and base',
      'Send joint angles with ri.angle_vector, wait with ri.wait_interpolation, drive with ri.go_pos, and read the robot back.'],
    ja: ['1. 基本: 関節と台車',
      'ri.angle_vector で関節角度を送り、ri.wait_interpolation で待ち、ri.go_pos で台車を動かして、ロボットの状態を読み返します。'],
  },
  {
    file: '02_pick.py',
    en: ['2. Pick a cube',
      'Drive up to a table, reach with inverse kinematics and lift a cube, asking ri where the cube is.'],
    ja: ['2. 箱をつかむ',
      '台まで進み、逆運動学で手を伸ばして箱を持ち上げます。箱の位置は ri から読みます。'],
  },
  {
    file: '03_robot_vs_ri.py',
    en: ['3. robot vs ri',
      'The model you plan with (blue) against the robot ri moves: plan a hand inside a table and see where the robot stops.'],
    ja: ['3. robot と ri の違い',
      '計画に使う模型（青）と、ri が動かすロボットの違い。手を机の中に入れる姿勢を送り、ロボットがどこで止まるかを見ます。'],
  },
  {
    file: '04_pick_robot_vs_ri.py',
    en: ['4. robot vs ri in a pick',
      'The pick again, stopping at each place robot and ri differ: planning, driving, grasping and where the cube is.'],
    ja: ['4. ピックで見る robot と ri の違い',
      'もう一度ピックしながら、robot と ri がずれる場面を順に見ます: 計画、台車の移動、把持、箱の位置。'],
  },
];

const LANGS = Object.keys(TEXT);

function storedLang() {
  try { return localStorage.getItem('web-ri-lang'); } catch (e) { return null; }
}

function initialLang() {
  const fromUrl = new URLSearchParams(location.search).get('lang');
  if (LANGS.includes(fromUrl)) return fromUrl;
  const stored = storedLang();
  if (LANGS.includes(stored)) return stored;
  return navigator.language && navigator.language.startsWith('ja') ? 'ja' : 'en';
}

let lang = initialLang();
const t = (key, ...args) => {
  const value = TEXT[lang][key];
  return typeof value === 'function' ? value(...args) : value;
};

const $ = (id) => document.getElementById(id);
const log = $('log'), statusEl = $('status'), statusBox = $('status-box');
const runButton = $('run'), stopButton = $('stop'), sampleSelect = $('sample');

// The last status, kept as a key so it can be shown again in another
// language.
let statusKey = ['loadingPython'], statusState = 'loading';
function setStatus(state, key, ...args) {
  statusState = state;
  statusKey = [key, ...args];
  statusBox.dataset.state = state;
  statusEl.textContent = t(key, ...args);
}

function applyLanguage() {
  document.documentElement.lang = lang;
  for (const el of document.querySelectorAll('[data-i18n]')) {
    el.textContent = t(el.dataset.i18n);
  }
  for (const el of document.querySelectorAll('[data-i18n-title]')) {
    el.title = t(el.dataset.i18nTitle);
  }
  for (const button of document.querySelectorAll('.lang button')) {
    button.setAttribute('aria-pressed', String(button.dataset.lang === lang));
  }
  const selected = sampleSelect.selectedIndex;
  sampleSelect.textContent = '';
  SAMPLES.forEach((s, i) => sampleSelect.add(new Option(s[lang][0], String(i))));
  sampleSelect.selectedIndex = Math.max(selected, 0);
  $('sample-desc').textContent = SAMPLES[sampleSelect.selectedIndex][lang][1];
  setStatus(statusState, ...statusKey);
  updateDocsLink();
}

// --- editor --------------------------------------------------------------

const editor = CodeMirror.fromTextArea($('editor'), {
  mode: 'python',
  lineNumbers: true,
  indentUnit: 4,
  matchBrackets: true,
  extraKeys: {
    Tab: (cm) => cm.execCommand(cm.somethingSelected() ? 'indentMore' : 'insertSoftTab'),
    'Shift-Tab': 'indentLess',
    'Ctrl-Enter': () => run(),
    'Cmd-Enter': () => run(),
  },
});
let loadedText = '';

async function loadSample(index) {
  const response = await fetch(`samples/${lang}/${SAMPLES[index].file}`);
  loadedText = await response.text();
  editor.setValue(loadedText);
  editor.clearHistory();
  $('sample-desc').textContent = SAMPLES[index][lang][1];
}

const edited = () => editor.getValue() !== loadedText;

// --- 3D view -----------------------------------------------------------

const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0.55, 0.65, 0.75);
addLights(scene);
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
camera.up.set(0, 0, 1);  // MuJoCo is z-up
camera.position.set(-0.5, -0.9, 0.6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.2, 0, 0.15);
controls.update();

const models = new Map();  // model id -> { root, objects, desc, ghost }
let shown = null;

// Until the program's first frame arrives the view would just sit empty:
// converting a detailed robot takes seconds.
const preparing = $('preparing');
function setPreparing(on) { preparing.style.display = on ? 'flex' : 'none'; }

function resize() {
  const { clientWidth: w, clientHeight: h } = view;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

// Frame everything but the floor when the scene's size changes a lot, and
// otherwise keep the viewpoint the student chose.
const VIEW_DIRECTION = new THREE.Vector3(-1.2, -2.2, 1.4).normalize();
let fittedSize = null;
function fitView(objects, force = false) {
  const box = new THREE.Box3();
  for (const o of objects) {
    if (o === null || o.userData.isPlane) continue;
    o.updateMatrixWorld(true);
    box.expandByObject(o);
  }
  if (box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3()).length();
  if (!force && fittedSize !== null && size < 1.5 * fittedSize
      && size > fittedSize / 1.5) return;
  fittedSize = size;
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(VIEW_DIRECTION, 1.3 * size);
  camera.near = size / 100;
  camera.far = size * 100;
  camera.updateProjectionMatrix();
  controls.update();
}

function showModel(id) {
  if (shown !== null) {
    const old = models.get(shown);
    scene.remove(old.root);
    if (old.ghost) scene.remove(old.ghost.root);
  }
  const m = models.get(id);
  scene.add(m.root);
  if (m.ghost) scene.add(m.ghost.root);
  shown = id;
}

$('fit').onclick = () => {
  if (shown !== null) fitView(models.get(shown).objects, true);
};

// --- camera image --------------------------------------------------------

const cameraCanvas = $('camera-canvas');
function showCamera({ camera: name, width, height, rgb }) {
  cameraCanvas.width = width;
  cameraCanvas.height = height;
  const image = new ImageData(width, height);
  for (let i = 0, j = 0; i < width * height; i++, j += 3) {
    image.data[4 * i] = rgb[j];
    image.data[4 * i + 1] = rgb[j + 1];
    image.data[4 * i + 2] = rgb[j + 2];
    image.data[4 * i + 3] = 255;
  }
  cameraCanvas.getContext('2d').putImageData(image, 0, 0);
  $('camera-name').textContent = name;
  $('camera').style.display = 'block';
}

// --- output --------------------------------------------------------------

function write(text, cls) {
  const line = document.createElement('span');
  if (cls) line.className = cls;
  line.textContent = text + '\n';
  log.appendChild(line);
  log.scrollTop = log.scrollHeight;
}

// --- worker --------------------------------------------------------------

let worker = null;
let running = false;

function setRunning(value) {
  running = value;
  if (!value) setPreparing(false);
  runButton.disabled = value;
  stopButton.disabled = !value;
}

function startWorker() {
  worker = new Worker('./worker.js', { type: 'module' });
  runButton.disabled = true;
  worker.onmessage = ({ data: msg }) => {
    switch (msg.type) {
      case 'status': setStatus('loading', msg.key, ...(msg.args || [])); break;
      case 'ready':
        runButton.disabled = false;
        setStatus('ready', 'ready');
        break;
      case 'stdout': write(msg.text); break;
      case 'stderr': write(msg.text, 'err'); break;
      // Shown with its first frame: until then every geom sits at the
      // origin.
      case 'model': {
        const built = buildModel(msg.desc);
        built.desc = msg.desc;  // the ghost is built from it on demand
        built.ghost = null;
        models.set(msg.id, built);
        break;
      }
      // The robot model the program plans with (robot), drawn translucent
      // over the simulated robot (ri).
      case 'ghost': {
        const m = models.get(msg.id);
        if (!m) break;
        if (m.ghost === null) {
          m.ghost = buildModel(m.desc, { only: msg.geomIds, ghost: true });
          if (shown === msg.id) scene.add(m.ghost.root);
        }
        setPoses(m.ghost.objects, msg.xpos, msg.xmat);
        break;
      }
      case 'frame': {
        const m = models.get(msg.id);
        if (m) {
          setPreparing(false);
          if (shown !== msg.id) showModel(msg.id);
          setPoses(m.objects, msg.xpos, msg.xmat);
          if (!m.posed) {
            m.posed = true;
            fitView(m.objects);
          }
        }
        break;
      }
      case 'camera': showCamera(msg); break;
      case 'done':
        setRunning(false);
        setStatus('done', 'done');
        break;
      case 'error':
        write(msg.text, 'err');
        setRunning(false);
        setStatus('error', 'error');
        break;
      case 'fatal':
        write(msg.text, 'err');
        setStatus('error', 'failed');
        break;
    }
  };
}

function run() {
  if (running || runButton.disabled) return;
  log.textContent = '';
  $('camera').style.display = 'none';
  setRunning(true);
  setPreparing(true);
  setStatus('running', 'running');
  worker.postMessage({ type: 'run', code: editor.getValue() });
}

// A blocked Python program cannot be interrupted without cross-origin
// isolation, so stopping means starting a fresh worker (and Python).
function stop() {
  worker.terminate();
  setRunning(false);
  write(t('stopped'), 'note');
  setStatus('loading', 'loadingPython');
  startWorker();
}

// --- controls ------------------------------------------------------------

let currentSample = 0;
sampleSelect.onchange = () => {
  if (edited() && !confirm(t('discard'))) {
    sampleSelect.selectedIndex = currentSample;
    return;
  }
  currentSample = sampleSelect.selectedIndex;
  loadSample(currentSample);
};
runButton.onclick = run;
stopButton.onclick = stop;
$('revert').onclick = () => {
  if (!edited() || confirm(t('discard'))) loadSample(sampleSelect.selectedIndex);
};
$('clear').onclick = () => { log.textContent = ''; };
for (const button of document.querySelectorAll('.lang button')) {
  button.onclick = () => {
    if (button.dataset.lang === lang) return;
    if (edited() && !confirm(t('discard'))) return;
    lang = button.dataset.lang;
    try { localStorage.setItem('web-ri-lang', lang); } catch (e) { /* private mode */ }
    const url = new URL(location.href);
    url.searchParams.set('lang', lang);
    history.replaceState(null, '', url);
    applyLanguage();
    loadSample(sampleSelect.selectedIndex);
  };
}

// The guide is the repository's README, in the page's language.
function updateDocsLink() {
  $('docs-link').href = lang === 'ja'
    ? 'https://github.com/iory/scikit-robot-web/blob/main/README.ja.md'
    : 'https://github.com/iory/scikit-robot-web#readme';
}

applyLanguage();
loadSample(0);
startWorker();
