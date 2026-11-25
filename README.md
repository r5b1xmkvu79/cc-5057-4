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

内核按**接口清楚的阶段**组织，主流程在 `src/core/pipeline.js` 的
`buildPipeline(project)`；`src/core/build.js` 的 `buildProject(project)`
是为旧入口保留的兼容适配，内部直接调用同一管线，因此旧样例与新界面结果完全
一致。阶段之间只传不可变的阶段产物，每个产物都带**来源（stage/source）与
单位（米、弧度、比率等）**的溯源元数据：

1. **输入规范化**（`stages/input.js`）：冻结默认控制参数，规范化节点、连接、
   地形与路口高程，产出不可变的项目输入。
2. **道路中心线求解**（`stages/routing.js`）：在连接节点之间，按道路等级、
   纵坡上限、曲率与禁入区域规划路线（`planner.js` 的 Theta* + 样条），并对
   急弯与地面争用选择「加宽」或「改线」；沿路线求解满足最大纵坡的纵断面，
   在路口两端压平（`taperEnds` 后再 `clampProfile`）。
3. **路口连接**（`stages/junction.js`）：生成端口、渐变段与带状路面，路口由
   路口平面高度场（端口残差 RBF 融合）三角化成真实路口面，带状路面被裁剪到
   路口外缘，二者精确相接、不重叠。
4. **网格生成**（`stages/mesh.js`）：组装整幅路面并发布可碰撞网格，检测重叠
   与未覆盖斑块、逐连接测量接缝。
5. **验收**（`stages/acceptance.js`）：路线可行性与整幅扫掠走廊分别验证
   （四角足迹离面点数、转向角、平面高差）；路口平面纵坡、端口高差与接缝
   单独检查。
6. **地形整型**（`stages/terrain-stage.js`）与**构建记录**（`stages/record.js`）：
   输出确定性构建记录（种子、控制参数、固定控制点、图号、修订号与溯源）。

### 修订号、编辑守卫与行车记录

路线与网格解耦是重构的边界，由三个 DOM 无关的小组件维护，并由
`src/core/geometry-interface.js` 的 `RoadGeometryInterface` 统一暴露给界面：

- **修订号台账**（`revision.js`）：中心线每次重算都递增道路修订号；碰撞网
  发布时按修订号交接，过期网格会被拒绝（`stale-mesh`），连续拖动后只有与
  最新道路同修订号的车辆轨迹才生效。
- **编辑守卫**（`edit-guard.js`）：试车轮胎尚压在旧桥面时，有接触的连接会被
  保留（held），未被占用的支路可以立即换网；登记安全停车点后可请求显式送回
  （parked → 确认后 free）。接触有宽限窗口，车辆驶离后自动释放。
- **行车记录**（`drive-log.js`）：只追加的逐帧记录，每帧记下当时的道路/网格
  修订号与轮胎实际采用的路面来源（道路带状面或路口面、连接/路口 id、实测
  高度，单位米）；道路更新后旧帧标记为过期。空路口重复编辑时，可从记录判断
  何时可通车以及实际采用的是哪块路面。
- 局部编辑只改作者侧的项目输入并整体重解，**不直接修改求解器内部数组**；
  已发布的路线几何保持不变。
- 界面查询路面高度或转弯可行性时都走真实计算（碰撞索引实测），离路面返回
  `null` 而不是伪造高度，未知路口直接抛错，异常由编辑器界面提示。

```js
import { RoadGeometryInterface } from './src/core/geometry-interface.js';
import { sampleProject } from './src/project.js';

const iface = new RoadGeometryInterface(sampleProject('arterial'));
console.log(iface.revision, iface.built.turnTests.map((t) => t.pass));
iface.planEdit(iface.built.routes.map((r) => r.conn.id)); // 编辑前的占用判定
```

### 等价性差分

`src/core/equivalence.js` 在净目录中对两次构建做实测差分：逐连接的中心线
平面/高程最大与 RMS 误差、逐连接接缝间隙差、路口三角面与面积对比。误差与
接缝全部由内核现算，不依赖任何手改数据，因此不允许伪报。多路口与陡坡样例的
无改动重解必须在 1e-6 m 容差内完全等价。

## 命令行（仅辅助）

道路构造的主入口是上面的图形编辑器；命令行只用于在终端里调用同一内核做
检查，不能替代编辑视窗：

```bash
node bin/roadforge.js build --scene arterial            # 构建摘要
node bin/roadforge.js diff --scene slope --conn branch-north --dx 6
node bin/roadforge.js open-junction --scene arterial    # 空路口通车与实际路面
```

## 验证

```bash
npm test        # 内核单元测试（Node，无浏览器）
npm run e2e     # 启动 Chromium 走查三个样例 + 驾驶 + 控制点拖动
```

`npm test` 覆盖：三样例零错误且所有转弯可行、每段都有带状路面、路口是真实
曲面、路面高度单值连续、确定性、构建记录、锁定重解，以及分阶段管线与旧入口
逐点等价、溯源与单位、求解器数组不被局部编辑改动、过期网格拒绝、拖动后轨迹
失效、编辑守卫三态、行车记录的实测路面来源、实测转弯、陡坡坡度/曲率规则、
多路口与陡坡等价差分、接缝对应与空路口通车判定。

### 无系统浏览器依赖时的 e2e

`npm run e2e` 需要 Playwright 的 Chromium 可正常启动。若系统缺少 Chromium 的
共享库，可把包含这些库的目录用 `ROADFORGE_SYSLIBS`（多个目录用 `:` 分隔）
指给运行器，或安装 `npx playwright install --with-deps chromium` 后直接运行。
e2e 默认访问 `http://localhost:5173/`，可用 `BASE_URL` 覆盖；运行前先
`npm run dev`。

## 目录

```
index.html            编辑器页面骨架
src/main.js           状态、重建流程、面板与指针/键盘交互（接 RoadGeometryInterface）
src/project.js        三个样例场景（节点、连接、地形、禁入区）
src/view/scene.js     Three.js 渲染、拾取、驾驶与高度采样
src/view/style.css    界面样式
src/core/             纯 JS 道路构造内核（可在 Node 中运行）
  pipeline.js         分阶段总装：输入→中心线→路口→网格→验收→地形→记录
  build.js            旧入口兼容适配（内部调用 pipeline.js）
  geometry-interface.js  界面侧统一接口：修订台账、编辑守卫、行车记录
  revision.js         道路/网格修订号交接
  edit-guard.js       轮胎接触占用与安全停车点
  drive-log.js        逐帧行车记录与实际采用路面
  provenance.js       来源、阶段与单位的溯源元数据
  equivalence.js      中心线误差与接缝的前后差分
  stages/             各阶段实现（input/routing/junction/mesh/acceptance/…）
  planner.js          Theta* 路由
  road.js             样条、纵断面、扫掠与加宽
  geometry.js         带状网格、路口高度场、转向轨迹、扫掠验收
  terrain.js          地形与道路走廊整型
  surface.js          三角化与可碰撞三角形索引
  clip.js             Clipper 多边形布尔
  standards.js        道路等级与车辆参数
bin/roadforge.js      命令行辅助（build / diff / open-junction）
test/kernel.test.js   内核测试
test/stages.test.js   分阶段管线、修订号、守卫、行车记录、等价差分测试
e2e/run.js            Playwright 走查
```
