# Road Forge — 道路构造关卡编辑器

一个浏览器端 3D 关卡编辑器：由纯 JavaScript 道路构造内核驱动，生成带宽度、
带高程和真实交叉口的路面网格，并提供可驾驶、可编辑控制点、可锁定重解的视窗。

内核完全在 `src/core/` 内，不依赖 DOM，可在 Node 中直接调用与测试；Three.js
只负责渲染同一个内核产出的几何。

## 启动界面

```bash
npm install
npm run dev
```

终端会打印地址，打开 **http://localhost:5173/** 即可（若端口被占用，Vite 会
自动改用下一个端口，以终端输出为准）。

生产构建与本地预览：

```bash
npm run build
npm run preview      # 默认 http://localhost:4173/
```

## 界面怎么用

- **场景切换**（顶栏）：三个样例 `主支路 T 形`、`跨坡地`、`近距离多岔`。
- **路线控制点**：下拉选择连接；绿色球为可拖动的内部控制点，白色球为连接
  节点端点。用鼠标拖动绿色球，松开鼠标后整条连接在内核中重新求解，其他连接
  一并更新。
- **锁定本段**：锁定当前连接的内部控制点；点「重新求解其余连接」时只保留本
  段，其余连接按同样的种子与约束重新路由。
- **车辆**：切换用于扫掠走廊验收的车型（sedan / boxTruck / bus）。
- **驾驶**：点「进入驾驶」，用 `W`/`↑` 油门、`S`/`↓` 刹车、`A`/`D` 转向，
  `Esc` 退出，`R` 重开到起点。车辆高度实时取用可碰撞路面网格，与渲染的
  路面一致；离开路面会被挡住而不是瞬移。
- **结果**：每条连接的长、最小半径、最大纵坡、加宽量与扫掠验收；每个路口的
  纵坡、端口高差与三角面数，以及每条转向轨迹的可行性（离面点数、最大转向、
  高差）。点转向行会在场景中显示该转向的扫掠走廊。
- **违规位置**：列出所有路线/路口/接缝的违规，点击定位并高亮；场景中以红/
  橙色锥体与光环标记。
- **构建记录**：显示种子与控制参数、固定控制点、路口与转弯判定；「复制
  JSON」可导出完整构建记录，用于锁定后复现。

## 内核能做什么

`src/core/build.js` 的 `buildProject(project)` 是唯一入口：

1. **路由**：在连接节点之间，按道路等级、纵坡上限、曲率与禁入区域规划路线
   （`planner.js` 的 Theta* + 样条），并对急弯与地面争用选择「加宽」或「改线」。
2. **高程**：沿路线求解满足最大纵坡的纵断面，在路口两端压平（`taperEnds` 后
   再 `clampProfile`），保证可见路面与行驶高度一致。
3. **路面**：生成带宽度的带状网格；路口由路口平面高度场（端口残差 RBF 融合）
   三角化成真实路口面，带状路面被裁剪到路口外缘，二者精确相接、不重叠。
4. **验收**：路线可行性与整幅扫掠走廊分别验证（四角足迹离面点数、转向角、
   平面高差）；路口平面纵坡、端口高差与接缝单独检查。
5. **记录**：输出确定性构建记录（种子、控制参数、固定控制点、路口与转弯判定）。

```js
import { buildProject } from './src/core/build.js';
import { sampleProject } from './src/project.js';

const built = buildProject(sampleProject('arterial'));
console.log(built.errorCount, built.turnTests.map((t) => t.pass));
```

## 验证

```bash
npm test        # 内核单元测试（Node，无浏览器）
npm run e2e     # 启动 Chromium 走查三个样例 + 驾驶 + 控制点拖动
```

`npm test` 覆盖：三样例零错误且所有转弯可行、每段都有带状路面、路口是真实
曲面、路面高度单值连续、确定性、构建记录、锁定重解。

### 无系统浏览器依赖时的 e2e

`npm run e2e` 需要 Playwright 的 Chromium 可正常启动。若系统缺少 Chromium 的
共享库，可把包含这些库的目录用 `ROADFORGE_SYSLIBS`（多个目录用 `:` 分隔）
指给运行器，或安装 `npx playwright install --with-deps chromium` 后直接运行。
e2e 默认访问 `http://localhost:5173/`，可用 `BASE_URL` 覆盖；运行前先
`npm run dev`。

## 目录

```
index.html            编辑器页面骨架
src/main.js           状态、重建流程、面板与指针/键盘交互
src/project.js        三个样例场景（节点、连接、地形、禁入区）
src/view/scene.js     Three.js 渲染、拾取、驾驶与高度采样
src/view/style.css    界面样式
src/core/             纯 JS 道路构造内核（可在 Node 中运行）
  build.js            总装：路由、路口、接缝、路面、验收、构建记录
  planner.js          Theta* 路由
  road.js             样条、纵断面、扫掠与加宽
  geometry.js         带状网格、路口高度场、转向轨迹、扫掠验收
  terrain.js          地形与道路走廊整型
  surface.js          三角化与可碰撞三角形索引
  clip.js             Clipper 多边形布尔
  standards.js        道路等级与车辆参数
test/kernel.test.js   内核测试
e2e/run.js            Playwright 走查
```
