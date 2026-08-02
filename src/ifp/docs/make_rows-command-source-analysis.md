# `make_rows` 命令解析——从零看懂,一路看到源码

> 这篇文档写给**第一次接触 floorplan(版图规划)的人**。我们从"这个命令是干嘛的"讲起,用一张图和一个真实例子建立直觉,再一步步深入 OpenROAD 源码(`src/ifp/` 模块),看它内部到底做了什么。
>
> 如果你只想要快速结论,先读 **第 0~3 节**;想了解实现,再读 **第 5~6 节**。
>
> 涉及的文件:
> - `src/ifp/src/InitFloorplan.tcl` —— Tcl 层(命令注册、参数解析、分发)
> - `src/ifp/src/InitFloorplan.i` —— SWIG 桥接层(Tcl 与 C++ 的接口)
> - `src/ifp/src/InitFloorplan.cc` —— C++ 层(核心实现)
> - `src/ifp/test/make_rows.tcl` 与 `make_rows_no_rows.def` —— 回归测试及输入

---

## 0. 这个命令是干什么的?(30 秒版)

芯片上要放成千上万个小单元(标准单元,比如反相器、触发器)。这些单元不能随便乱放,它们必须**整整齐齐地排成一行一行**——每一行宽窄一致、彼此对齐,这样单元才能恰好落在一个规则的网格上,布线才能顺畅。

`make_rows` 干的事就是:**在芯片的某个区域里,画出这一行一行的位置**,供后续的布局(placement)使用。

可以想象成在操场上给班级划做操站位:先定好一块区域(核心区),再在里面画出一条条跑道(行),每条跑道上用粉笔打满等距的小格子(站点)。`make_rows` 就是"画跑道+打格子"这个动作。

```{note}
它画的是"行"。至于**格子打多大、跑道画多高**,由工艺库里的 **site**(站点)决定;画在哪片区域,由 **core**(核心区)决定;区域外面还有一层 **die**(芯片外框)。这四个词是本节后面马上要讲的。
```

什么时候用 `make_rows`?典型场景:**拿到一份只有芯片边界、没有行的版图文件(DEF)**,想补上放置行。比如 OpenROAD 自带测试 `make_rows_no_rows.def` 就是这种情况——文件里有 die、有单元、有连线,唯独没有 `ROW` 语句。

---

## 1. 先认识四个词:Die、Core、Site、Row

这四个词是理解本文全部内容的基础。我们把它们从大到小排好:

```
┌──────────────────────────────────────────────┐
│  Die —— 芯片最外框(整颗芯片的边界)            │
│  ┌────────────────────────────────────────┐  │
│  │  Core —— 允许放标准单元的区域            │  │
│  │                                        │  │
│  │   ┌──┬──┬──┬──┬──┬──┬──┐              │  │
│  │   │  │  │  │  │  │  │  │ ← Row(一行)   │  │
│  │   └──┴──┴──┴──┴──┴──┴──┘              │  │
│  │   ┌──┬──┬──┬──┬──┬──┬──┐              │  │
│  │   │  │  │  │  │  │  │  │ ← Row(又一行) │  │
│  │   └──┴──┴──┴──┴──┴──┴──┘              │  │
│  └────────────────────────────────────────┘  │
│                                        │     │
└──────────────────────────────────────────────┘
       每个小格 = 一个 Site(一个单元格子)
```

逐个解释:

| 概念 | 一句话 | 类比 | 在代码里的实体 |
| --- | --- | --- | --- |
| **Site** | 一个标准单元占的最小"格子",宽高固定 | 停车位 | `odb::dbSite` |
| **Row** | 一行拼在一起的格子,数量 = 格宽能放几个 | 一溜停车位 | `odb::dbRow` |
| **Core** | 放所有行的矩形区域 | 停车场的划线范围 | `odb::Rect` |
| **Die** | 芯片最外层边界 | 停车场围墙 | `odb::dbBlock::getDieArea()` |

**Site 是谁定的?** 由工艺库(LEF 文件)定义。比如 OpenROAD 测试用的 `FreePDK45_38x28_10R_NP_162NW_34O`,它规定格子的宽和高(在该测试里实测为 380 DBU 宽、2800 DBU 高)。一个 site 就是一个"最小放置单位",单元尺寸通常是它的整数倍。

**Row 为什么重要?** 布局阶段,每个标准单元必须"坐在"某一条 row 上,并且占的位置是 site 的整数倍。没有 row,单元就不知道往哪放。所以 `make_rows` 是布局前的必要准备。

**Core 和 Die 什么关系?** Die 是整个芯片,Core 是中间一块允许放标准单元的区域(宏单元、I/O 等会占掉 Die 边缘和别的区域)。Core 通常比 Die 小一圈,中间的空隙留给电源环、宏单元等。

---

## 2. 看一次真实运行

我们用 OpenROAD 自带的回归测试 `src/ifp/test/make_rows.tcl` 来感受一下。它做的事就三行:

```tcl
read_lef Nangate45/Nangate45.lef        ;# 读工艺库,获得 site 定义
read_liberty Nangate45/Nangate45_typ.lib
read_def make_rows_no_rows.def          ;# 读一份"没有行"的版图
make_rows -core_space 1 \
          -site FreePDK45_38x28_10R_NP_162NW_34O    ;# 生成行!
write_def make_rows.def
```

### 运行前:版图里没有行

输入文件 `make_rows_no_rows.def` 里只有芯片边界和单元,没有 `ROW`:

```
UNITS DISTANCE MICRONS 2000 ;                    ;# 单位:1 micron = 2000 DBU
DIEAREA ( 0 0 ) ( 28280 18140 ) ;                ;# Die:左下(0,0) 右上(28280,18140)
COMPONENTS 5 ;                                   ;# 5 个单元
    - r1 DFF_X1 ;                                ;#   触发器、缓冲器、与门…
    - r2 DFF_X1 ;
    ...
END COMPONENTS
;  (注意:没有任何 ROW_xx 语句)
```

### 运行后:版图里多了 4 行

输出文件 `make_rows.defok` 里出现了 4 条 `ROW`:

```
DIEAREA ( 0 0 ) ( 28280 18140 ) ;
ROW ROW_0 FreePDK45_38x28_10R_NP_162NW_34O 2280 2800 N DO 63 BY 1 STEP 380 0 ;
ROW ROW_1 FreePDK45_38x28_10R_NP_162NW_34O 2280 5600 FS DO 63 BY 1 STEP 380 0 ;
ROW ROW_2 FreePDK45_38x28_10R_NP_162NW_34O 2280 8400 N DO 63 BY 1 STEP 380 0 ;
ROW ROW_3 FreePDK45_38x28_10R_NP_162NW_34O 2280 11200 FS DO 63 BY 1 STEP 380 0 ;
```

### 一行 `ROW` 语句怎么看

以 `ROW_0` 为例,逐字段解读(单位都是 DBU):

```
ROW ROW_0   site名                        2280   2800   N   DO 63 BY 1  STEP 380 0
│    │      │                              │      │     │       │            │
│    │      │                              │      │     │       │            └ site 宽 380,高间隔 0 → 每个格子宽 380
│    │      │                              │      │     │       └ DO 63 BY 1 → 沿 x 方向 63 个格
│    │      │                              │      │     └ 方向 N(见第 3.4 节)
│    │      │                              │      └ 行的左下角 y = 2800
│    │      │                              └ 行的左下角 x = 2280
│    │      └ 用哪个 site(格子大小)
│    └ 行的名字(第 0 行)
└ 关键字 ROW
```

对比四行的 y 坐标:2800 → 5600 → 8400 → 11200,每行相差 **2800**,正好是一个 site 的高度。也就是说:一行贴着一行往上排。

---

## 3. 跟着算一遍:这 4 行是怎么来的?

这一节我们只用手算 + 常识,重现 `make_rows` 的全部决策过程。这是理解整个命令的关键,也是理解源码的钥匙。

**已知条件:**
- Die 左下角 (0, 0),右上角 (28280, 18140),单位 DBU;
- 1 micron = 2000 DBU;
- 命令给了 `-core_space 1`,即四周各留 1 micron = 2000 DBU 的空隙;
- site 宽 380、高 2800 DBU(来自 LEF,从输出能反推出来)。

**第一步:算出 Core 区域**

Core = Die 四条边各自向内缩 2000 DBU:

```
Core 左下角 = (0+2000, 0+2000)          = (2000, 2000)
Core 右上角 = (28280-2000, 18140-2000)  = (26280, 16140)
```

**第二步:把 Core 左下角"对齐"到 site 网格(snap)**

格子宽 380,那么 Core 的左边最好正好是 380 的整数倍,这样行才从格子边界开始。2000 不是 380 的整数倍,所以要**向上取整**到最近的倍数:

```
x:  ceil(2000 / 380) × 380 = 6 × 380 = 2280
y:  ceil(2000 / 2800) × 2800 = 1 × 2800 = 2800
```

所以每行真正从 **(2280, 2800)** 开始——和输出里 ROW_0 的坐标完全一致。这个"悄悄把角落挪到网格上"的动作,源码里叫 **snap**(对齐),如果发生了会打一条警告。

**第三步:算每行放几个格子、一共放几行**(都是整数除法,放不下的部分直接舍弃)

```
每行格子数 = (26280 - 2280) / 380 = 24000 / 380 = 63.15… → 63   ✓(DO 63)
行数       = (16140 - 2800) / 2800 = 13340 / 2800 = 4.76…  → 4    ✓(ROW_0..3)
```

**第四步:方向怎么定?**

标准单元顶部/底部有电源轨(VDD/VSS)。如果所有行都朝一个方向,相邻两行的电源轨会"背靠背挤在一起",浪费高度。所以正确的做法是**让行交替朝向**:第 0 行朝 N,第 1 行朝 FS,第 2 行朝 N……这样相邻行的轨道可以共用、更省面积。看输出,`N, FS, N, FS`,完全吻合。

> 新手友好解释:`N` = 正常方向,`FS` = 水平镜像。它俩的差别只是单元"头朝上还是头朝下",不影响功能,只影响电源轨怎么并在一起。`-flip_sites` 参数可以把某个 site 的起始方向反过来(第 4 节)。

**第五步:命名**

依次叫 `ROW_0`、`ROW_1`、`ROW_2`、`ROW_3`。

---

到这里,你已经完全理解了 `make_rows` 的输出是怎么来的。下面才进入命令本身和源码。

---

## 4. 命令语法与参数

```tcl
make_rows
  (-core_area {llx lly urx ury}) | (-core_space (space | {bottom top left right}))
  -site site_name
  [-additional_sites site_names]
  [-flip_sites site_names]
  [-gap space]
  [-row_parity NONE|EVEN|ODD]
```

| 参数 | 必须? | 新手向解释 | 技术说明 |
| --- | --- | --- | --- |
| `-core_area {llx lly urx ury}` | 二选一 | 直接告诉命令"行画在这个矩形里",单位微米 | 与 `-core_space` 互斥;必须恰好 4 个坐标 |
| `-core_space space` | 二选一 | 不直接给矩形,而是说"从 Die 四周各缩进多少",缩完剩下的矩形就是 Core | 可给 1 个值(四边相同)或 4 个值,顺序 `{bottom top left right}` |
| `-site site_name` | **必填** | 用哪个格子(LEF 里的 site 名)来排 | 决定行高 = site 高、格宽 = site 宽 |
| `-additional_sites` | 否 | 还想给其它 site 也建行(比如将来会用到的高个子单元) | 会与基础 site 一起生成行 |
| `-flip_sites` | 否 | 让这些 site 的起始行方向反过来(FS/N 互换) | 默认第 0 行 N,第 1 行 FS;列在这里的 site 变成第 0 行 FS、第 1 行 N |
| `-gap` | 否 | 有电压域(多电压)设计时,域与普通区之间留的间距 | 默认 6 × 最小 site 高度;必须为正数 |
| `-row_parity` | 否 | 把行数强制成偶数或奇数 | 实现是"只减不加":行数不符就少建一行 |

```{note}
单位说明:凡是用微米给的参数(`-core_area`、`-core_space`、`-gap`),内部都会乘上工艺的 DBU 换算因子(本测试里 1 micron = 2000 DBU)。源码里到处是 DBU 整数,是因为版图数据统一用 DBU 存储、避免浮点误差。
```

---

## 5. 内部流程:一句话概括每一步

不管参数怎么给,`make_rows` 内部都是按下面这条流水线走:

```
① 检查参数合法(-site 缺了?gap 是负数?-core_area 和 -core_space 都给了?)
② 确定 Core 区域(-core_area 直接用;否则 Die 四边缩 -core_space)
③ 检查 Die 存在、包含 Core;检查每个单元都能塞进 Core
④ 决定"给哪些 site 建行"(基础 + 附加 + 网表里用到的),并删掉所有旧行
⑤ 把 Core 左下角对齐到 site 网格(snap)
⑥ 按 site 高度从下往上铺行:宽 = 格宽,高 = 行高,方向 N/FS 交替
⑦ 若有电压域,把穿过域边界的行拆成"左段/域内/右段",域间留 -gap
⑧ 有放置障碍(blockage)的地方,把行剪开(avoid 掉)
⑨ 打印 Die/Core 面积、利用率等统计
```

第 ④ 步有个新手容易忽略的点:**`make_rows` 每次都会先删光旧行再重建**。所以它叫 "make" 而不是 "add"——重复执行结果不变(幂等)。

---

## 6. 深入源码

现在把第 5 节的每一步,对应到真实代码。命令横跨三层:TCLL 脚本 → SWIG 胶水 → C++ 实现。

### 6.1 调用链总览

```
[用户]  make_rows -core_space 1 -site xxx
  │
  ▼
[Tcl]  proc make_rows                      InitFloorplan.tcl:74    解析参数 → 交给 helper
  ▼
[Tcl]  proc make_rows_helper               InitFloorplan.tcl:82    校验 + 单位换算 + 分发
  │    ├─ parse_row_params                  InitFloorplan.tcl:256  解析 -site/-additional_sites/-flip_sites/-row_parity/-gap
  │    └─ 有 -core_area  → ifp::make_rows(4 坐标)                  InitFloorplan.tcl:104
  │       有 -core_space → ifp::make_rows_with_spacing(4 边距)      InitFloorplan.tcl:136
  ▼
[SWIG] ifp::make_rows / make_rows_with_spacing   InitFloorplan.i:130/152  类型适配后调 C++
  ▼
[C++] InitFloorplan::makeRows / makeRowsWithSpacing   InitFloorplan.cc:448/405  核心实现
```

### 6.2 Tcl 层:解析参数(第 ① 步)

命令入口(`InitFloorplan.tcl:74`):

```tcl
proc make_rows { args } {
  sta::parse_key_args "make_rows" args \
    keys {-core_space -core_area -site -additional_sites -row_parity -flip_sites -gap} \
    flags {}
  make_rows_helper [array get keys]
}
```

`make_rows_helper` 里真正做事(Tcl 层的"第 ① 步 + 第 ② 步"):

```tcl
lassign [ifp::parse_row_params keys] site additional_sites flipped_sites row_parity gap

if { [info exists keys(-core_area)] } {
  if { [info exists keys(-core_space)] } {
    utl::error IFP 60 "-core_space cannot be used with -core_area."   ;# 互斥检查
  }
  lassign $keys(-core_area) core_lx core_ly core_ux core_uy           ;# 4 个坐标
  ...
  ifp::make_rows \
    [ord::microns_to_dbu $core_lx] [ord::microns_to_dbu $core_ly] \   ;# 微米→DBU
    ...
}
if { [info exists keys(-core_space)] } {
  ... # 1 或 4 个边距,同样转成 DBU
  ifp::make_rows_with_spacing ...
}
utl::error IFP 62 "no -core_area or -core_space specified."           ;# 两个都没给
```

`parse_row_params`(`InitFloorplan.tcl:256`)是 `initialize_floorplan` 和 `make_rows` **共用**的参数解析器:`-site` 必填(缺了报 IFP 35),`-row_parity` 只收 `NONE/ODD/EVEN`(否则 IFP 57),`-gap` 没给时用一个特殊哨兵值 `-(2**31)`(= `INT32_MIN`)占位——这个哨兵的含义在第 6.4 节第 ⑦ 步揭晓。

### 6.3 SWIG 层:类型适配(第 ③ 步之前的"搬运工")

SWIG(`InitFloorplan.i:130`)生成的函数几乎不含逻辑,只是把数据"掰成" C++ 方便的形状,再调 C++ 成员函数。值得一提的唯一一件事:

```cpp
odb::PtrSet<odb::dbSite> flipped_sites_set(flipped_sites.begin(),
                                           flipped_sites.end());
```

把 `-flip_sites` 的 `std::vector` 转成 `PtrSet`(一种集合),方便后面快速判断"这个 site 要不要翻转方向"。

### 6.4 C++ 核心:逐段对应

**① 参数与前置检查** —— `makeRows`(`InitFloorplan.cc:448`)开头:

```cpp
checkGap(gap);                                    // gap <= 0 → 报错 IFP 36
odb::Rect block_die_area = block_->getDieArea();
if (block_die_area.area() == 0) {                 // die 面积为 0 → IFP 63
  logger_->error(IFP, 63, "Floorplan die area is 0. Cannot build rows.");
}
if (!block_die_area.contains(core)) {             // die 没包住 core → IFP 55
  logger_->error(IFP, 55, "Die area must contain the core area.");
}
checkInstanceDimensions(core);                    // 有单元塞不进 core → IFP 2
```

第 ④ 步的"删旧行 + 选 site 集合"在 `prepareSitesAndClearRows`(`InitFloorplan.cc:242`):

```cpp
sites_by_name[base_site->getName()] = base_site;          // ① 基础 site
if (base_site->hasRowPattern()) { ... }                   // ② 行模式里的 site
for (auto site : additional_sites) { ... }                // ③ -additional_sites
addUsedSites(sites_by_name);                              // ④ 网表里用到的 site
// 删除所有旧行:
for (auto row_itr = rows.begin(); row_itr != rows.end();)
  row_itr = dbRow::destroy(row_itr);
```

> 第 ④ 步"网表里用到的 site"(`addUsedSites`,`InitFloorplan.cc:688`):命令会自动扫描设计里**已经实例化**的标准单元用了哪些 site,并为它们也建行——所以即使你没写 `-additional_sites`,只要网表里用了高个子单元,它的行也会自动出现。用 `std::map<string,dbSite*>` 去重,同名只留一个。

**② snap + 铺行** —— `makeRows` 主体:

```cpp
if (core.xMin() >= 0 && core.yMin() >= 0) {        // Core 左下角必须非负才继续
  eval_upf(network_, logger_, block_);             // 读取 UPF 电压域信息
  // 对齐到 site 网格(divCeil = 向上取整除法):
  const int clx = divCeil(core.xMin(), site_dx) * site_dx;   // 2000 → 2280
  const int cly = divCeil(core.yMin(), site_dy) * site_dy;   // 2000 → 2800
  ...
  if (clx != core.xMin() || cly != core.yMin()) {  // 发生移动 → 警告 IFP 28
    logger_->warn(IFP, 28, "Core area lower left ... snapped ...");
  }
  if (base_site->hasRowPattern())
    makeHybridRows(...);                           // 混合高度设计
  else
    makeUniformRows(...);                          // 常规设计 ← 本例走这里
  updateVoltageDomain(clx, cly, cux, cuy, gap);   // ⑦ 电压域拆分
}
```

`makeUniformRows`(`InitFloorplan.cc:713`)就是第 ③ 步算行数 + 第 ⑥ 步铺行的实现,和我们第 3 节手算的一模一样:

```cpp
const int rows_x = core_dx / site_dx;             // 每行格子数:整数除法
const int rows_y = applyRowParity(core_dy / site_dy, row_parity);  // 行数(奇偶约束)

for (int row = 0; row < rows_y; row++) {
  // 偶数行 N(R0),奇数行 FS(MX);flip=true 时互换
  dbOrientType orient = ((row + flip) % 2 == 0) ? dbOrientType::R0
                                                : dbOrientType::MX;
  dbRow::create(block_, row_name, site, core.xMin(), y, orient,
                dbRowDir::HORIZONTAL, rows_x, site_dx);   // 固定水平行
  y += site_dy;                                     // 往上挪一行的高度
}
```

`applyRowParity`(`InitFloorplan.cc:388`)实现"只减不加"的奇偶约束:

```cpp
case RowParity::kEven: rows_y = (rows_y / 2) * 2; break;   // 奇数则减 1
case RowParity::kOdd:  if (rows_y > 0 && rows_y % 2 == 0) rows_y--; break;
```

**⑦ 电压域拆分** —— `updateVoltageDomain`(`InitFloorplan.cc:538`):如果设计定义了电压域/功耗域(`VOLTAGE_DOMAIN`/`POWER_DOMAIN` 组),一条横穿域边界的行会被拆成三段:左段 `<行名>_1`、域内段 `<行名>_<域名>`、右段 `<行名>_2`,中间隔开的间距就是 `-gap`。这里揭晓哨兵值的用法:

```cpp
// gap 未指定(哨兵 INT32_MIN)时,默认取 6 倍最小 site 高度
const int power_domain_y_space
    = (gap == std::numeric_limits<int32_t>::min()) ? 6 * min_site_dy : gap;
```

**⑧ 剪开障碍** —— 行生成完毕后:

```cpp
for (auto blockage : block_->getBlockages())
  blockage_bboxes.push_back(blockage->getBBox());
odb::cutRows(block_, /* min_row_width */ 0, blockage_bboxes,
             /* halo_x */ 0, /* halo_y */ 0, logger_);
```

`odb::cutRows`(实现在 odb 模块)把与放置障碍(blockage,如宏单元占位)重叠的行剪断,保证行不穿过障碍。

**⑨ 统计** —— `reportAreas`(`InitFloorplan.cc:1365`)打印 Die/Core 包围盒、Core 面积、实例总面积、有效利用率、实例数(信息码 IFP 100~105)。

### 6.5 两条特殊路径

- **混合高度设计(hybrid rows)**——`makeHybridRows`(`InitFloorplan.cc:836`):当基础 site 自身带"行模式"(row pattern,比如"1 个矮 + 1 个高"循环)时,铺行不再每个 site 独立,而是按 pattern 循环取 (site, 方向),直到高度用尽。此时不允许 `-row_parity`(IFP 51)。附加的高 site 通过 `getOffset`(`InitFloorplan.cc:780`)在基础 pattern 里找自己的对齐起点。
- **`-core_area` 与 `-core_space` 两条路径**其实殊途同归:`-core_space` 在 `makeRowsWithSpacing`(`InitFloorplan.cc:405`)里先把 Die 四边缩边距算出 core 矩形,然后调用同一个 `makeRows`。所以从 `makeRows` 往下的逻辑两者完全一致。

---

## 7. 常见问题(FAQ)

**Q1: 为什么行要 N / FS 交替?**
标准单元内部有电源轨。如果两行同向,相邻边界上是两条同类型的轨道挤在一起,浪费高度;交替方向后相邻行共享一条轨道,更省面积。`-flip_sites` 只是把某个 site 的"第 0 行方向"从 N 换成 FS。

**Q2: 我给的 Core 左下角是 (2000,2000),为什么输出行从 (2280,2800) 开始?**
因为行必须从 site 网格的整数边界开始。2000 不是 site 宽(380)/高(2800)的整数倍,命令把它向上取整到 2280 / 2800(见第 3 节第二步),并打印一条 "snapped" 警告。

**Q3: 我算出来能放 4.76 行,为什么只有 4 行?**
行数用整数除法 `core_height / site_height` 计算,不足一整行高度的剩余空间直接舍弃。同理,每行格数也是整除。

**Q4: `-gap` 到底管什么?**
只和电压域有关:当设计里定义了多个电压域,`-gap` 决定普通区与电压域之间的行间距(默认 6 × 最小 site 高度)。没有电压域时它不产生任何影响。

**Q5: `make_rows` 和 `initialize_floorplan` 有什么区别?**
`initialize_floorplan` 是"一站式":创建 Die、创建 Core、建行、建轨道都做。`make_rows` 只做"建行"这一件事,且**不会**改 Die、**不会**重建轨道、**会**先删掉旧行。场景对上了就用它,比如拿到的 DEF 没带行。

**Q6: 为什么行只生成在 Core 左下角非负时?**
源码里整个铺行逻辑都包在 `core.xMin() >= 0 && core.yMin() >= 0` 里。Core 跑到负坐标属于异常输入,此时不生成任何行(也是一种防御)。

**Q7: 我用了 `-additional_sites`,为什么没建行?**
检查两点:① 该 site 名是否存在于已读入的 LEF 中;② 附加 site 高度是否是基础 site 高度的整数倍(不是整数倍会报 IFP 54)。

---

## 8. 错误与警告速查

| 码 | 级别 | 含义 | 见 |
| --- | --- | --- | --- |
| IFP 28 | 警告 | Core 左下角被对齐到 site 网格(坐标被挪了) | Q2 |
| IFP 36 | 错误 | `-gap` 必须为正 | §6.4① |
| IFP 55 | 错误 | Die 区域没有包含 Core 区域 | §6.4① |
| IFP 61 | 警告 | 某个 site 一行都没生成 | — |
| IFP 62 | 错误 | 既没给 `-core_area` 也没给 `-core_space` | §6.2 |
| IFP 63/64 | 错误 | Die 面积为 0,无法建行 | §6.4① |
| IFP 65 | 错误 | Core 区域里一行都没生成 | — |
| IFP 2 | 错误 | 某个单元塞不进 Core | §6.4① |

完整清单以 `src/ifp/messages.txt` 为准。

---

## 9. 回归测试

`src/ifp/test/make_rows.tcl` 是最直接的覆盖:`read_def make_rows_no_rows.def`(无行版图)→ `make_rows -core_space 1 ...` → `write_def` → 与 golden 文件 `make_rows.defok` 逐字节比对。本文第 2、3 节用的就是这套数据。

`init_floorplan_gap.tcl` 则专门覆盖 `-gap` 的校验:给 `-gap -1`、`-gap 0` 都应报"必须为正"(IFP 36)。

---

## 10. 小结

`make_rows` 虽小,却串起了一条完整的分层流水线:

1. **概念上**,它回答"单元往哪放"的第一步——在 Core 里按 site 铺出 N/FS 交替的水平行。
2. **数学上**,它的核心就是三次整除:core 左下角对齐到网格、每行格数、总行数,外加一个"只减不加"的奇偶修正。
3. **工程上**,它由 Tcl(参数解析)→ SWIG(类型适配)→ C++(算法)三层实现,核心 `makeRows` 干五件事:校验 → 选 site/删旧行 → 对齐 → 铺行 → 收尾(电压域拆分 + 剪障碍 + 统计)。

看懂 `make_rows`,你就同时看懂了 `initialize_floorplan` 里"建行"那一半——因为两者共用同一个 `makeRows` 实现。
