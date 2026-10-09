# initialize_floorplan 中文源码笔记：从画边界到读懂建行算法

配套：[交互式 HTML 教程](initialize_floorplan-explained.html)。直接用浏览器打开即可，无须联网或安装依赖。

本文面向第一次接触 OpenROAD floorplan 的读者，依据本仓库提交 `a55db75eb1` 的源码编写。学习顺序是：认识对象 → 运行命令 → 手算输出 → 跟踪源码 → 理解多高度、多边形和实现边界。文中的行为结论以当前源码为准；`.ok` 和 `.defok` 是仓库保存的预期输出，不等于在所有版本上重新运行的结果。

## 1. 这条命令完成什么工作？

网表告诉我们“有哪些单元、它们怎么连接”，却没有完整的物理放置场地。`initialize_floorplan` 为设计建立芯片边界，并在指定的核心区域里生成标准单元放置行。

可以把它想成规划一个停车场：先确定围墙，再确定内部划线区域，最后画出一排排车位。此时还没有给每辆车安排位置。

| 概念 | 在这里的含义 | 主要源码对象 |
| --- | --- | --- |
| Die | 整个芯片的边界，可以是矩形或正交多边形 | `dbBlock` 中保存的 die polygon |
| Core | 本次请求用来生成 rows 的区域；最终数据库中的 core 又与生成的 rows 有关 | `odb::Rect` / `odb::Polygon`，`dbBlock::setCoreArea()` |
| Site | LEF 定义的放置格子，具有宽、高和其他属性 | `odb::dbSite` |
| Row | 由同一 site 沿水平方向重复形成的一行 | `odb::dbRow` |
| Track | 金属布线的参考轨道，与放置行用途不同 | `odb::dbTrackGrid` |

一个标准单元通常横跨多个 site，不应把一个 site 理解为一个完整逻辑单元。`initialize_floorplan` 也没有完成 placement、I/O 引脚安排、电源网格生成或布线。

特别要记住：建 die 时会清除旧 routing tracks，建 rows 时会删除旧 rows。需要轨道时，在初始化之后另行执行 `make_tracks`。重复执行初始化会改写已有 floorplan；错误发生后也不能假定数据库自动恢复到调用前的状态。

## 2. 第一层：先会写一个矩形 floorplan

以下例子使用仓库提供的 Nangate45 数据，从 **`src/ifp/test` 目录**运行。LEF 提供物理尺寸和 site；Liberty 提供时序库；Verilog 提供逻辑网表；`link_design top` 将顶层设计关联起来。

```tcl
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_verilog reg1.v
link_design top

initialize_floorplan \
  -die_area {0 0 1000 1000} \
  -core_area {100 100 900 900} \
  -site FreePDK45_38x28_10R_NP_162NW_34O

make_tracks
write_def /tmp/initialize_floorplan-rectangle.def
```

花括号中的四个数是 `{左下x 左下y 右上x 右上y}`，单位为微米（µm），不是 `{x y 宽 高}`。这个例子请求一个 1000 × 1000 µm 的 die，并在中间 800 × 800 µm 的 core 中建行。

为什么必须有花括号或引号？因为 Tcl 要把四个坐标作为同一个参数。Site 名也必须与已读入的 LEF 定义一致；不要把 README 示例中的 `-sites` 拼写照搬过来，实际选项是单数 **`-site`**。

对应的现有测试是 [init_floorplan1.tcl](../test/init_floorplan1.tcl)。其 [预期日志](../test/init_floorplan1.ok) 给出：

```text
Core area lower left (100.000, 100.000) snapped to (100.130, 100.800).
Added 570 rows of 4209 site FreePDK45_38x28_10R_NP_162NW_34O.
Core BBox: (100.130, 100.800) 到 (899.840, 898.800) um
```

请求边界与输出边界不同，是网格对齐与整数取整的结果，下一节会逐步算出来。

## 3. 第二层：让利用率决定尺寸

当还没有固定 die 尺寸时，可以使用：

```tcl
# 已完成上一节的 read_* 和 link_design
initialize_floorplan \
  -utilization 30 \
  -aspect_ratio 0.5 \
  -core_space 1 \
  -site FreePDK45_38x28_10R_NP_162NW_34O
```

这就是 [init_floorplan2.tcl](../test/init_floorplan2.tcl) 的主体。

`-utilization 30` 表示以 30% 为尺寸估算目标，参数输入是百分数。`-aspect_ratio 0.5` 表示 **高度 / 宽度 = 0.5**，所以 core 比较扁。`-core_space 1` 表示 core 四周各留 1 µm。四边不同时，顺序固定为：

```tcl
-core_space {bottom top left right}
# 例如 -core_space {2 3 4 5}：下2、上3、左4、右5 µm
```

设实例面积总和为 `A`，利用率百分数为 `U`，高宽比为 `r`，忽略整数取整时：

```text
u = U / 100
目标 core 面积 = A / u
core 宽度 W = sqrt(A / u / r)
core 高度 H = W × r
die = (0, 0) → (left + W + right, bottom + H + top)
```

固定面积和比例时，利用率降低会使 core 变大；固定面积和利用率时，比例增大会使 core 变高、变窄。增大 `core_space` 主要扩大 die，不会在连续公式中改变目标 core 面积，但离散网格上的行数可能变化。

### 3.1 `designArea()` 到底统计什么？

实现遍历 `block_->getInsts()`，累加每个实例对应 `dbMaster` 的 `width × height`。它使用 LEF/OpenDB 的物理尺寸，不读取 Liberty 的 `area` 属性，也没有在这个函数里专门过滤宏、pad 或 cover。

因此，不应把此处的 `A` 无条件解释成“只统计标准单元面积”，也不应把利用率估算视为布局一定能完成的证明。宏能否放下、行是否足够、拥塞和合法化可行性，是不同问题。

### 3.2 为什么输入 30%，日志却是 0.372？

该测试有 5 个实例，总面积 `15.428 µm²`。LEF 给出：

```text
DATABASE MICRONS 2000       → 1 µm = 2000 DBU
MANUFACTURINGGRID 0.0050    → 制造网格 = 10 DBU
SITE SIZE 0.19 BY 1.4       → site = 380 × 2800 DBU
```

DBU 是数据库距离单位。距离乘 2000，面积则乘 `2000²`，不要混用。

| 步骤 | 计算或结果 |
| --- | --- |
| 连续目标面积 | `15.428 / 0.30 = 51.426666… µm²` |
| 连续宽高 | `W ≈ 10.141663 µm`，`H ≈ 5.070832 µm` |
| C++ 整数宽高 | 宽度截断为 `20283 DBU`，高度按该宽度计算后四舍五入为 `10142 DBU` |
| 未对齐 die 右上角 | `(24283, 14142) DBU` |
| 制造网格对齐后的 die | `(0,0) → (24280,14140) DBU`，即 `12.140 × 7.070 µm` |
| 从已保存 die 内缩四边 | 请求 core 为 `(2000,2000) → (22280,12140) DBU` |
| 左下角向上对齐 site | `x = ceil(2000/380)×380 = 2280`；`y = ceil(2000/2800)×2800 = 2800` |
| 每行 site 数 | `floor((22280−2280)/380) = 52` |
| 行数 | `floor((12140−2800)/2800) = 3` |
| rows 外包框面积 | `(52×0.19) × (3×1.4) = 41.496 µm²` |
| 日志的有效利用率 | `15.428/41.496 ≈ 0.371796 → 0.372`，即约 `37.2%` |

日志打印的是比值 `0.372`，不是 `0.372%`。本例的连续目标 core、传给建行函数的 core、实际 rows 外包框，是三个相关但不相同的量。

注意这里有两种对齐，方向不同：die 坐标对制造网格**就近取整**；rows 左下角对 site 网格**向上取整**。右边和上边靠整数除法舍弃不足一个格子或一行的空间。

从 [init_floorplan2.defok](../test/init_floorplan2.defok) 还能直接核对：

```def
DIEAREA ( 0 0 ) ( 24280 14140 ) ;
ROW ROW_0 FreePDK45_38x28_10R_NP_162NW_34O 2280 2800 N DO 52 BY 1 STEP 380 0 ;
ROW ROW_1 FreePDK45_38x28_10R_NP_162NW_34O 2280 5600 FS DO 52 BY 1 STEP 380 0 ;
ROW ROW_2 FreePDK45_38x28_10R_NP_162NW_34O 2280 8400 N DO 52 BY 1 STEP 380 0 ;
```

`DO 52 BY 1` 表示一行重复 52 个 site；`STEP 380 0` 是这一行内部重复 site 的步长，不是相邻两行的 y 间距。行高应查看 site，或比较各行 y 坐标。`N` 对应 C++ `R0`；`FS` 对应 `MX`，即关于 x 轴镜像、上下翻转。

## 4. 第三层：沿真实调用链读源码

### 4.1 先找 Tcl 入口，不要只搜索同名 C++ 函数

[InitFloorplan.tcl](../src/InitFloorplan.tcl) 中，`sta::define_cmd_args` 注册参数帮助，`proc initialize_floorplan` 才是脚本入口。

```text
initialize_floorplan（Tcl）
  ├─ sta::parse_key_args：解析参数和做部分互斥校验
  ├─ die 或 core 坐标数 > 4？
  │    ├─ 是：ifp::make_polygon_die_helper → ifp::make_polygon_rows_helper
  │    └─ 否：ifp::make_die_helper         → make_rows_helper
  │
  └─ 矩形路径展开
       ├─ 有 utilization → ifp::make_die_util → makeDieUtilization → makeDie
       ├─ 有 die_area    → ifp::make_die      → makeDie
       └─ 建行
            ├─ core_area  → ifp::make_rows → makeRows
            └─ core_space → ifp::make_rows_with_spacing
                            → makeRowsWithSpacing → makeRows
```

图中 `ifp::make_*` 是 SWIG 导出的 Tcl 命令，后面的驼峰名称是 C++ 方法。当前 C++ 确实存在两个 `InitFloorplan::initFloorplan()` 重载，但上述 Tcl 入口将 die 和 rows 分开调用。读到同名函数便停止追踪，会遗漏实际参数验证顺序。

### 4.2 SWIG 桥接层做了什么？

[InitFloorplan.i](../src/InitFloorplan.i) 把 Tcl 参数转换为 C++ 参数，再调用 `design->getFloorplan()` 返回的对象。

| Tcl 可见调用 | C++ 方法 |
| --- | --- |
| `ifp::make_die_util` | `InitFloorplan::makeDieUtilization` |
| `ifp::make_die` | `InitFloorplan::makeDie` |
| `ifp::make_rows_with_spacing` | `InitFloorplan::makeRowsWithSpacing` |
| `ifp::make_rows` | `InitFloorplan::makeRows` |
| `ifp::make_polygon_die` | `InitFloorplan::makePolygonDie` |
| `ifp::make_polygon_rows_simple` | `InitFloorplan::makePolygonRows` |

Tcl helper 通过 `ord::microns_to_dbu` 把距离转换为 DBU；多边形坐标列表再由 SWIG 转为 `std::vector<odb::Point>`。`RowParity` 有专用 typemap；`flipped_sites` 从列表变为指针集合。

桥接函数签名中有 `ord::Design* design`，Tcl 却不传它，是因为 [src/Design.i](../../Design.i) 的 `numinputs=0` typemap 从解释器关联数据中取得当前设计。不要误以为 Tcl 少传了一个参数。另一个容易误读的地方是 `make_rows_with_spacing` 的桥接形参名：应沿位置追到 C++，它们实际按 **下、上、左、右** 传递。

### 4.3 C++ 怎样写入 die？

[InitFloorplan.cc](../src/InitFloorplan.cc) 中的核心操作可摘录为：

```cpp
// makeDieUtilization：此时面积单位是 DBU²。
utilization /= 100;
const double core_area = designArea() / utilization;
const int core_width = std::sqrt(core_area / aspect_ratio);
const int core_height = round(core_width * aspect_ratio);

// makeDie：各坐标经 snapToMfgGrid 后组成 die_area。
block_->setDieArea(die_area);
resetTracks();
```

`resetTracks()` 遍历并销毁 block 中原有 `dbTrackGrid`。这解释了为何需要先 `initialize_floorplan` 再 `make_tracks`。`makeRowsWithSpacing()` 随后读取**已经对齐并保存的 die**，内缩四边得到 core，而不是直接保留前面计算出的浮点目标 core。

### 4.4 C++ 怎样生成 rows？

普通矩形路径中，`makeRows()` 依次完成：

1. 校验 `gap`、die 面积和 die 包含 core；对实例尺寸做初步检查。
2. `prepareSitesAndClearRows()` 收集需要的 sites，并删除所有旧 rows。
3. 对非负的 core 左下角路径执行 `eval_upf()`，并将左下角对齐到基础 site 网格。
4. 根据基础 site 是否具有 `ROWPATTERN`，调用 `makeUniformRows()` 或 `makeHybridRows()`。
5. `updateVoltageDomain()` 处理电压域间距和行分割，再由 `odb::cutRows()` 按 placement blockages 裁行。
6. `reportAreas()` 输出面积和实例统计。

`checkInstanceDimensions()` 是尺寸的初步筛查，跳过 pad/cover，并对允许 R90 对称的 master 使用特殊尺寸比较；它不是完整的旋转放置、拥塞或合法性求解器。

普通单 site 场景的核心伪代码如下，假设坐标非负、范围有效：

```text
clx = ceil(core.xMin / site.width)  × site.width
cly = ceil(core.yMin / site.height) × site.height
columns = floor((core.xMax − clx) / site.width)
rows    = applyRowParity(floor((core.yMax − cly) / site.height))

对 row_index = 0 .. rows−1：
    y = cly + row_index × site.height
    orient = ((row_index + flip) % 2 == 0) ? N : FS
    dbRow::create(block, name, site, clx, y, orient,
                  HORIZONTAL, columns, site.width)
```

方向交替与库中的电源轨和行使用约定有关；不能把这个操作解释为已经生成了电源网络。

## 5. 第四层：附加参数何时有用？

| 参数 | 默认或含义 | 当前实现要点 |
| --- | --- | --- |
| `-site` | 必填的基础 site | 找不到时报错；帮助中虽有方括号，建行 helper 仍要求它 |
| `-aspect_ratio` | `1.0`；高度/宽度 | 使用正数；C++ 没有限制必须 ≤ 1 |
| `-core_space` | 利用率模式要求显式给出 | 可为 0；单值或 `{下 上 左 右}` 四值 |
| `-additional_sites` | 空列表 | 为之后可能使用的单元提前准备其他 site rows |
| `-flip_sites` | 空列表 | 普通 rows 将指定 site 的 N/FS 序列反转 |
| `-row_parity` | `NONE` | 仅接受大写 `NONE`、`EVEN`、`ODD` |
| `-gap` | 内部使用未指定哨兵值 | 电压域处理时默认 `6 × 最小 site 高度`；显式值转换到 DBU 后必须 > 0 |

### 5.1 行数奇偶：只减少，不增加

`applyRowParity()` 不扩张 core。原始行数 5 配 `EVEN` 得 4；原始行数 4 配 `ODD` 得 3；0 行不会因 `ODD` 变为 1 行。约束针对生成阶段的各 site 行数，经过障碍物或电压域分割后，数据库里的最终 row 对象数不一定仍满足该奇偶性。

```tcl
initialize_floorplan \
  -die_area {0 0 1000 1000} \
  -core_area {100 100 900 898} \
  -site FreePDK45_38x28_10R_NP_162NW_34O \
  -row_parity EVEN \
  -flip_sites {FreePDK45_38x28_10R_NP_162NW_34O}
```

这时请求范围先能容纳 569 行，`EVEN` 再减少为 568 行；第 0 行方向改为 FS。这里演示的两个选项分别可参考 [even rows 测试](../test/init_floorplan_even_rows.tcl) 和 [flip sites 测试](../test/init_floorplan_flip_sites.tcl)。

### 5.2 多高度与 hybrid rows 是两种机制

普通多高度流程收集基础 site、附加 sites 和网表中合适实例使用的 sites。`addUsedSites()` 只对符合 `isCoreAutoPlaceable()` 且不是 block 的实例加入 site，按名称去重。各 site 的高度必须是基础 site 高度的整数倍；生成不同高度的 rows 可以提供重叠的候选放置资源，并不代表实例已经叠放。

实现细节：矩形 `makeUniformRows()` 用基础 site 宽度决定所有 rows 的 x 步长和格子数，用各 site 自己的高度决定 y 步长。因此不要把它想成任意宽度 site 的通用拼砖算法。

Hybrid rows 则由基础 site 的 `ROWPATTERN` 决定各子 site 及方向。`getOffset()` 通过模式匹配计算其他 pattern 的偏移，必要时尝试翻转模式。当前矩形 hybrid 路径不支持 `row_parity != NONE`，也不把 `flipped_sites` 传给 `makeHybridRows()`；不能照搬普通行的 flip 规则。参考 [hybrid_rows.tcl](../test/hybrid_rows.tcl)、[multi_height1.tcl](../test/multi_height1.tcl)。

### 5.3 电压域、gap 与 blockages

`-gap` 不是“每两条普通行之间的间距”。它在 `updateVoltageDomain()` 中作用于已定义的电压域周围：切掉接近域边界的行段，再按域内、域外重建行。无相关电压域时，调大 gap 不会简单地把整个 core 的 rows 均匀拉开。

普通 placement blockages 则在末尾交给 `odb::cutRows()`。因此，一开始的规则 rows 可以变成不同长度的多个片段。参考 [init_floorplan_gap.tcl](../test/init_floorplan_gap.tcl)、[upf_test.tcl](../test/upf_test.tcl)、[placement_blockage1.tcl](../test/placement_blockage1.tcl)。

## 6. 第五层：非矩形 floorplan 怎样建行？

当前 Tcl 入口只要发现 **die 或 core 任意一个坐标列表长度大于 4**，就进入多边形路径。进入后两者都必须提供，并各自至少有 8 个数（4 个顶点），坐标数必须为偶数。即使其中一个几何形状是矩形，也要写成 4 个顶点的 8 坐标形式。

“坐标数必须为偶数”不等于“顶点数必须为偶数”。应使用简单、非自交、轴对齐的正交多边形，相邻顶点共享 x 或 y，core 应完全位于 die 内。

```tcl
# 已读入库和网表，并 link_design。
# L 形 die 与内缩后的 L 形 core，单位 µm。
initialize_floorplan \
  -die_area  {0 0  200 0  200 100  100 100  100 200  0 200} \
  -core_area {20 20  180 20  180 80  80 80  80 180  20 180} \
  -site FreePDK45_38x28_10R_NP_162NW_34O
```

不要用 `-utilization` 或 `-core_space` 推导多边形 core；这里应该显式提供两个轮廓。

### 6.1 两层几何处理

第一层是 die。`makePolygonDie()` 对顶点做制造网格对齐，再调用 [dbBlock::setDieArea(Polygon)](../../odb/src/db/dbBlock.cpp)。OpenDB 保存真实 die polygon，同时把“die 外包矩形减去多边形”的空白区域转为系统保留的 placement/routing blockages。`getDieArea()` 返回的仍是外包矩形；要取真实轮廓，应看 `getDieAreaPolygon()`。

第二层是 core。`makePolygonRows()` 转到 `makePolygonRowsScanline()`，再针对各 site 调用 `makeUniformRowsPolygon()`。对每个候选 y 层级：

1. 建立一个横向候选 row 矩形。
2. `intersectRowWithPolygon()` 用 **row 底边的 y** 与多边形边求交，忽略水平边。
3. 将交点 x 排序，按偶奇规则两两配对为内部区间。
4. 各区间左端对齐 site 网格，宽度取能容纳的整数 site 数，创建 row 片段。
5. 最后继续做电压域处理和 blockages 裁行。

L 形在较低 y 处得到较长的 row，高于缺口后得到短 row。U 形可以在同一个 y 处得到两个片段；它们的方向由同一个 `row_idx` 决定。HTML 中可以拖动扫描线观察交点配对。

### 6.2 读源码时必须保留的边界条件

这里列的是当前实现边界，不是推荐用户依赖的特殊用法：

- `makePolygonRows()` 显式检查的是 die 外包框包含 core 外包框，并非完整的多边形包含关系；die 外部还有 OpenDB 系统 blockages 帮助裁切。不能把“没报错”当成 core 完全在 die 内的证明。
- `intersectRowWithPolygon()` 实际采样 row 底边，未对整个行高做严格多边形相交。若 core 的凹角高度落在一行中间，不能仅凭该函数推断整行都位于 core 内；die 的系统 blockages 也不等同于 core 缺口检查。
- 这条路径没有完整显式验证所有边均轴对齐、无自交等几何条件。有效输入应由调用方准备好，不应把校验遗漏当作支持斜边的承诺。
- 多边形路径暂不支持基础 site 带 `ROWPATTERN` 的 hybrid rows，会报 `IFP-1000`。
- `row_parity` 应用在候选 y 层级数上。一个层级可能生成多个片段或没有片段，所以不是对最终片段数量施加奇偶约束。

推荐对照 [make_polygon_rows1.tcl](../test/make_polygon_rows1.tcl) 至 [make_polygon_rows4.tcl](../test/make_polygon_rows4.tcl)。注意 [init_floorplan_polygon.tcl](../test/init_floorplan_polygon.tcl) 主要检查读写 polygon DEF 时系统障碍物不被写出，它不是直接调用本命令生成多边形 rows 的测试。

## 7. 参数和日志里最容易误读的地方

| 场景 | 如何理解或排查 |
| --- | --- |
| `IFP-0019` | 没给 `-utilization` 或 `-die_area` |
| `IFP-0034` | 利用率模式没给 `-core_space`，即使想留 0 也需显式写出 |
| `IFP-0035` | row 参数解析时缺少 `-site`；错误码需结合完整消息看，因为不同路径可能复用编号 |
| `IFP-0062` | 矩形建行没有 `-core_area` 或 `-core_space`；常见于只给 die |
| `IFP-0028` | core 左下角发生 site 对齐的警告，通常不是失败 |
| `IFP-0055` | 矩形 die 不包含请求 core |
| `IFP-0002` | 实例尺寸初筛失败，可看 [init_floorplan10.tcl](../test/init_floorplan10.tcl) |
| `IFP-0061` / `IFP-0065` | 某 site / 全部普通 rows 未生成；[init_floorplan12.tcl](../test/init_floorplan12.tcl) 演示过高利用率与扁平比例导致无行 |
| `IFP-0085` | 多边形模式缺少显式 core polygon |

推荐把参数分成三套清晰配置：显式矩形 die/core、利用率配 core_space、多边形 die/core，避免混搭。

还有几处文档与源码差异值得知道：

- README 的比例范围文字写到 1.0；当前 C++ 只要求 `aspect_ratio > 0`，大于 1 表示高而窄的 core。
- 推荐利用率为 `0 < U ≤ 100`；当前 `makeDieUtilization()` 仅显式检查非负，没有完整保护 `U=0` 和 `U>100`。教学页面主动限制有效输入，不能据源码校验缺口建议使用异常值。
- Tcl 名为 `check_positive_float` 的函数实际允许 0；真正限制还要往 C++ validator 追踪。因此不能仅看函数名判断范围。
- 矩形模式拒绝 `-die_area` 和 `-aspect_ratio` 同时出现；多边形分支提前分流，没有完全复用同一套检查，附加的 aspect_ratio 不会用来改变多边形。
- `reportAreas()` 取 `getCoreArea()` 的**外包矩形面积**作为日志分母；生成器中的 `setCoreArea(computeCoreArea())` 又发生在后续电压域/障碍物裁行之前。对于多边形、孔洞、分离的 rows 或裁行场景，不能把日志里的 “Core area” 和 “Effective utilization” 无条件等同于最终可放置 site 面积及其占用率。

## 8. 回看源码的阅读路线与验证方法

建议按照这个顺序阅读，先定位函数名，再阅读其调用者和被调用者：

| 文件 | 优先查找的符号 | 本文对应部分 |
| --- | --- | --- |
| [InitFloorplan.tcl](../src/InitFloorplan.tcl) | `initialize_floorplan`、`make_die_helper`、`make_rows_helper`、`parse_row_params` | 命令分流、默认值、校验顺序 |
| [InitFloorplan.i](../src/InitFloorplan.i) | `make_die_util`、`make_rows_with_spacing`、polygon typemap | 跨语言参数传递 |
| [InitFloorplan.hh](../include/ifp/InitFloorplan.hh) | `InitFloorplan`、`RowParity` | 类接口与参数类型 |
| [InitFloorplan.cc](../src/InitFloorplan.cc) | `designArea`、`makeDieUtilization`、`snapToMfgGrid`、`makeRows`、`makeUniformRows` | 面积、取整、建行 |
| [InitFloorplan.cc](../src/InitFloorplan.cc) | `makePolygonRowsScanline`、`intersectRowWithPolygon`、`reportAreas` | 多边形与统计口径 |
| [dbBlock.cpp](../../odb/src/db/dbBlock.cpp) | `setDieArea`、`getDieArea`、`computeCoreArea` | 数据真正怎样保存 |

已有测试脚本会生成结果并与 golden DEF 比较。例如，若仓库根目录下 `build/bin/openroad` 已构建：

```bash
# 从仓库根目录进入测试数据目录。
cd src/ifp/test
RESULTS_DIR=/tmp/ifp-note-results \
  ../../../build/bin/openroad -no_init -no_splash -exit init_floorplan2.tcl
```

`RESULTS_DIR` 避免输出写入输入文件旁；每个例子最好启动一个新的 OpenROAD 进程，避免上次初始化的状态影响观察。可用同样方式检查 `init_floorplan1.tcl`、`init_floorplan_even_rows.tcl`、`init_floorplan11.tcl`、`init_floorplan12.tcl` 和 `make_polygon_rows1.tcl`。第 11 个测试连续初始化两次，适合观察重建行为。

配套 HTML 的普通单 site 计算器按上文 DBU 整数公式计算，默认复现第 2 个算例；多边形画面是帮助理解源码的简化模型，未实现完整 OpenDB、UPF 或 blockage 行为。它不能替代 OpenROAD 运行结果。

### 本次文档核对记录

使用本机现有 `build/bin/openroad`（版本输出 `26Q3-1950-g26b1a08127`）运行了 `init_floorplan1.tcl`、`init_floorplan2.tcl`、`init_floorplan_even_rows.tcl`、`init_floorplan11.tcl`、`make_polygon_rows1.tcl`，均通过脚本内的 DEF golden 比较；`init_floorplan12.tcl` 按预期报告无行错误 `IFP-0065`。二进制版本标识与本文源码基准不同，因此这些运行结果用于核对所选算例，不据此声称二进制与当前源码完全一致。

HTML 已在 Chrome 中检查默认算例数值、五个建行阶段、奇偶与翻转选项、比例大于 1、无行及无效输入、源码面板、L/U 形扫描线，以及桌面和移动视口的页面宽度；未发现 JavaScript 异常。相对文件链接已核对存在。

延伸阅读：[make_rows 源码笔记](make_rows-command-source-analysis.md)、[make_tracks 源码笔记](make_tracks-command-source-analysis.md)、[dbSite 说明](dbSite-explained.md)。
