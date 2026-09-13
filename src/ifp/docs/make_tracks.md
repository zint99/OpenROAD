# `make_tracks` 命令解析——从零看懂,一路看到源码

> 这篇文档写给**第一次接触 floorplan / 布线的人**。我们从"这个命令是干嘛的"讲起,用一张图和一个真实例子建立直觉,再一步步深入 OpenROAD 源码(`src/ifp/` 模块),看它内部到底做了什么。
>
> 建议先读完讲 `make_rows` 的那篇兄弟文档(理解 Die/Core/Site/Row),再读本篇——本篇会反复拿 **Track** 和 **Row** 对比。
>
> 涉及的文件:
> - `src/ifp/src/InitFloorplan.tcl` —— Tcl 层(命令注册、参数解析)
> - `src/ifp/src/InitFloorplan.i` —— SWIG 桥接层
> - `src/ifp/src/InitFloorplan.cc` —— C++ 层(核心实现,`makeTracks` 一族函数)
> - `src/odb/src/defout/defout_impl.cpp` —— odb 把轨道写成 DEF `TRACKS` 语句的地方
> - `src/ifp/test/make_tracks1~7.{tcl,defok}` —— 回归测试

---

## 0. 这个命令是干什么的?(30 秒版)

上一份文档里,`make_rows` 画的是"**放置标准单元的行**"——单元坐上去的格子。本片的 `make_tracks` 画的是"**金属线走的轨道**"——布线时,每一条金属线的**中心线必须恰好压在一条轨道上**,不能想画哪画哪。

用车道类比:行是划好的**停车位**(车=单元要停进去),轨道是**车道线**(车流=金属线必须沿着线走,不能压线、不能越道)。

```
Row(上一份文档)                 Track(本份文档)
┌──┬──┬──┬──┐                   ────────   ← 一条轨道
│  │  │  │  │  ← 单元放这         ────────   ← 又一条轨道
└──┴──┴──┴──┘                    ────────   ← 金属线的中心线压在轨道上
```

`make_tracks` 做的事就是:**在每个金属层上,铺出两族等间距的平行轨道**(一族横、一族竖),供布线器(router)使用。

```{note}
一句话记忆:行(Row)管"**单元放哪**",轨道(Track)管"**线往哪走**"。两者单位不同:行由 site 决定,轨道由金属层的 pitch/offset 决定。
```

什么时候用 `make_tracks`?最常见的是配合 `make_rows` 一起给一份"空版图"补全基础设施:先用 `make_rows` 补行,再用 `make_tracks` 补轨道。

---

## 1. 先认识几个词:Layer、Track、Pitch、Offset

### 金属层 Layer

芯片不是一层金属,而是很多层(metal1、metal2、metal3…)上下堆叠,层与层之间用 via 连通。为了简化布线和避免干扰,**每层金属只能走一个固定方向**——通常是相邻层交替:

```
metal1:水平方向走线(HORIZONTAL)
metal2:垂直方向走线(VERTICAL)
metal3:水平方向走线(HORIZONTAL)
...
```

### 轨道 Track

一层金属上,布线器能下笔的"线"的位置。一条轨道就是一条直线(中心线位置)。布线器把金属线的中心对齐到轨道上,线与线之间就能保证最小间距。

### 轨道族:一横一竖

每一层金属都定义**两族**等距轨道:

```
      ┃        ┃        ┃        ┃        ┃     ← X 族:竖线,固定 x 坐标
      ┃        ┃        ┃        ┃        ┃
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   ← Y 族:横线,固定 y 坐标
      ┃        ┃        ┃        ┃        ┃
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
      ┃        ┃        ┃        ┃        ┃
      ┃        ┃        ┃        ┃        ┃
```

- **X 族**:x 坐标固定、沿 y 方向延伸的竖线,用于**竖直走向**的金属(如 metal2)。
- **Y 族**:y 坐标固定、沿 x 方向延伸的横线,用于**水平走向**的金属(如 metal1)。

### Pitch(节距/间距)与 Offset(偏移)

```
      offset ─┐
              ▼
      ┃   ┃   ┃   ┃   ┃   ┃   ┃
      │◄──►│
      pitch       ← 相邻两条轨道的中心距
```

- **Pitch**:相邻两条轨道中心线的距离。工艺上它至少是"线宽 + 最小线距",不能更密。
- **Offset**:第一条轨道离 Die 左下角的距离,也叫起点。

### 这些值从哪来?

两层来源:默认来自**工艺库(LEF)**,也可以被命令参数**临时覆盖**。以 Nangate45 库的 metal1 为例(LEF 原文):

```
LAYER metal1
  TYPE ROUTING ;
  WIDTH 0.07 ;          ← 最小线宽 0.07um
  PITCH 0.14 ;          ← pitch 0.14um = 280 DBU
  DIRECTION HORIZONTAL ;← 本层水平走线
  OFFSET 0.095 0.07 ;   ← offsetX 0.095um = 190 DBU,offsetY 0.07um = 140 DBU
END metal1
```

这就是下一节真实输出里 `STEP 280`、`TRACKS X 190`、`TRACKS Y 140` 的出处。

---

## 2. 看一次真实运行

我们看测试 `make_tracks2.tcl`,它对 metal1、metal2 显式给了 pitch 和 offset,最适合手算:

```tcl
initialize_floorplan -die_area "0 0 1000 1000" \
  -core_area "100 100 900 900" \
  -site FreePDK45_38x28_10R_NP_162NW_34O

make_tracks metal1 -x_offset 0.1 -x_pitch 0.2 -y_offset 0.1 -y_pitch 0.2
make_tracks metal2 -x_offset 0.1 -x_pitch 0.2 -y_offset 0.1 -y_pitch 0.2
```

运行后 DEF 里 metal1 出现了两条轨道语句(metal2 相同,略):

```
UNITS DISTANCE MICRONS 2000 ;                    ;# 1 micron = 2000 DBU
TRACKS X 200 DO 5000 STEP 400 LAYER metal1 ;
TRACKS Y 200 DO 5000 STEP 400 LAYER metal1 ;
```

### 一条 `TRACKS` 语句怎么看

以 `TRACKS X 200 DO 5000 STEP 400 LAYER metal1` 为例:

```
TRACKS X  200    DO 5000   STEP 400   LAYER metal1
│       │       │         │          └ 属于哪个金属层
│       │       │         └ 相邻两条间隔 400 DBU
│       │       └ 一共 5000 条
│       └ 第一条轨道在 x = 200
└ 轨道族方向:X = 竖直线族(固定 x);Y = 水平线族(固定 y)
```

也就是说:metal1 上有 5000 条竖线,第一条在 x=200,往后每 400 条……哦不对,是**每 400 DBU** 一条,共 5000 条。

---

## 3. 跟着算一遍:这 5000 条是怎么来的?

**已知条件:**
- Die 左下角 (0, 0),右上角 (1000, 1000),单位 um → 换算成 DBU 是边长 2000000;
- 命令给了 `-x_offset 0.1`、`-x_pitch 0.2`、y 同理 → offset = 200 DBU,pitch = 400 DBU;
- 该层最小线宽 0.07um → 半线宽 = 70 DBU(第 4 步的边界检查要用)。

**第一步:算第一条轨道的位置(origin)**

```
origin_x = Die.xMin + x_offset = 0 + 200 = 200
origin_y = Die.yMin + y_offset = 0 + 200 = 200
```

**第二步:算轨道条数(count)**

从第一条开始,每隔 pitch 一条,直到超出 Die 边界:

```
x 方向:count = (Die.dx() - x_offset) / x_pitch + 1
             = (2000000 - 200) / 400 + 1
             = 4999 + 1 = 5000
y 方向:同样 = 5000
```

**第三步:边界修剪(把贴着边的、画不了线的轨道去掉)**

检查第一条轨道:它的中心线离 Die 左边界必须至少**半个线宽**,否则这条线上放不下一条完整的金属线。检查最后一条同理。

```
X 族:第一条 x=200,200 - 70 = 130 ≥ 0 ✓
     最后一条 x = 200 + (5000-1)×400 = 1999800,1999800 + 70 ≤ 2000000 ✓
     → 不需要减,保持 5000 ✓
Y 族:同样保持 5000 ✓
```

**结果:** 和输出完全一致——`TRACKS X 200 DO 5000 STEP 400`、`TRACKS Y 200 DO 5000 STEP 400`。✓

### 进阶:为什么有的层是 7142 而不是 7143?

再看测试 `make_tracks1.tcl`——它**不带任何参数**,所以用 LEF 默认值(metal1:pitch 280、offsetX 190、offsetY 140):

```
TRACKS X 190 DO 7142 STEP 280 LAYER metal1 ;
TRACKS Y 140 DO 7143 STEP 280 LAYER metal1 ;
```

同样三步,但这次 offsetX(190)比 offsetY(140)大:

```
X 族:count = (2000000 - 190) / 280 + 1 = 7142 + 1 = 7143
     最后一条 x = 190 + 7142×280 = 1999950
     1999950 + 70 = 2000020 > 2000000  ← 超界!贴着右边界,画不下完整的线
     → 减一条,得 7142 ✓
Y 族:count = (2000000 - 140) / 280 + 1 = 7143
     最后一条 y = 140 + 7142×280 = 1999900
     1999900 + 70 = 1999970 ≤ 2000000  ← 刚好不超界
     → 保持 7143 ✓
```

这就是为什么同一个层,X 族是 7142 条、Y 族是 7143 条——**只差在 offset 那 50 DBU 上**。边界修剪是理解轨道数量最关键的一环。

> 补充:如果 Die 左下角不在 (0,0)(比如测试 3 里是 (10,20)),`origin` 会从 Die 左下角**算起**:x = 10000+… 反正公式不变,只是 Die.xMin 不再是 0。

---

到这里,你已经完全理解了 `make_tracks` 的输出是怎么来的。下面进入命令本身和源码。

---

## 4. 命令语法与参数

```tcl
make_tracks
    [layer]
    [-x_offset x_offset]
    [-y_offset y_offset]
    [-x_pitch x_pitch]
    [-y_pitch y_pitch]
```

| 参数 | 必须? | 新手向解释 | 技术说明 |
| --- | --- | --- | --- |
| `[layer]` | 否 | 只给这一层金属铺轨道;不给就**所有路由层**都铺 | 参数必须是 ROUTING 类型的层,否则报错 |
| `-x_offset` | 否 | 第一条**竖线**离 Die 左边界的距离(微米) | 不填用 LEF 里该层的 OFFSET;必须非负 |
| `-y_offset` | 否 | 第一条**横线**离 Die 下边界的距离(微米) | 同上 |
| `-x_pitch` | 否 | 竖线族里相邻两条的间距(微米) | 不填用 LEF 里该层的 PITCH;必须为正 |
| `-y_pitch` | 否 | 横线族里相邻两条的间距(微米) | 同上 |

```{note}
- `x_*` 管竖线族(X 族),`y_*` 管横线族(Y 族)——不要被"X 是横"的直觉误导,规则是:名字带 x 的就是 x 坐标固定、间隔按 x 方向的轨道。
- 参数只对**指定的那一层**生效。不给层时,`-x_pitch` 等参数不能用(见 FAQ Q5)。
- 单位微米,内部会先对齐到制造网格(mfg grid)再转 DBU。
```

---

## 5. 内部流程:一句话概括每一步

`make_tracks` 内部按这条流水线走(以"只铺一层"为例):

```
① 确认这一层存在、且是可布线的 ROUTING 层
② 确定 pitch/offset:命令给了就用命令的,没给就从 LEF 读(该层的 PITCH/OFFSET)
③ 算第一条轨道位置 origin = Die起点 + offset
④ 算条数 count = (Die尺寸 - offset) / pitch + 1
⑤ 边界修剪:第一条/最后一条轨道若离 Die 边界不足半个线宽,就整体让位或减一条
⑥ 把 (起点, 条数, 间距) 记进这一层的轨道网格(track grid)
⑦ 不指定层时:对每一层 ROUTING 层重复②~⑥
```

对比 `make_rows`:`make_rows` 每次都**先删旧行**;`make_tracks` **不删旧轨道**——它只把轨道网格填进去。删轨道是 `initialize_floorplan` 干的事(它建 Die 时会清空轨道)。

---

## 6. 深入源码

命令同样横跨 TCL → SWIG → C++ 三层。

### 6.1 调用链总览

```
[用户]  make_tracks metal1 -x_offset 0.1 -x_pitch 0.2 -y_offset 0.1 -y_pitch 0.2
  │
  ▼
[Tcl]  proc make_tracks                      InitFloorplan.tcl:158
  │    ├─ 校验层存在(IFP 10)/是 ROUTING 层(IFP 25)
  │    ├─ 每个参数:给了→ 对齐到制造网格;没给 → 读 LEF(getPitchX/getOffsetX…)
  │    └─ ifp::make_layer_tracks(layer, x_offset, x_pitch, y_offset, y_pitch)
  │       (不给层 → ifp::make_layer_tracks 无参版)
  ▼
[SWIG] ifp::make_layer_tracks                InitFloorplan.i:124 / 176
  ▼
[C++] InitFloorplan::makeTracks(layer,…)     InitFloorplan.cc:1027   核心算法
      InitFloorplan::makeTracks()            InitFloorplan.cc:987    遍历所有层
      InitFloorplan::makeTracksNonUniform()  InitFloorplan.cc:1117   LEF58_PITCH 特殊层
  ▼
[odb]  dbTrackGrid::create / addGridPatternX/Y + DefOut 写 DEF TRACKS 语句
```

### 6.2 Tcl 层:解析参数、决定默认值(第 ①② 步)

命令入口(`InitFloorplan.tcl:158`):

```tcl
proc make_tracks { args } {
  sta::parse_key_args "make_tracks" args \
    keys {-x_pitch -y_pitch -x_offset -y_offset} flags {}
  sta::check_argc_eq0or1 "make_tracks" $args      ;# 最多带一个位置参数:层名

  if { [llength $args] == 0 } {
    ifp::make_layer_tracks                         ;# 不给层 → 所有层
  } elseif { [llength $args] == 1 } {
    set layer [$tech findLayer $layer_name]
    if { $layer == "NULL" } {
      utl::error "IFP" 10 "layer $layer_name not found."
    }
    if { [$layer getType] != "ROUTING" } {
      utl::error "IFP" 25 "layer $layer_name is not a routing layer."
    }
    # 每个参数:pitch/offset "给了就用,没给就读 LEF"
    if { [info exists keys(-x_pitch)] } {
      set x_pitch [ifp::microns_to_mfg_grid $keys(-x_pitch)]   ;# 微米→制造网格→DBU
    } else {
      set x_pitch [$layer getPitchX]               ;# 默认值从 LEF 来
    }
    if { [info exists keys(-x_offset)] } {
      set x_offset [ifp::microns_to_mfg_grid $keys(-x_offset)]
    } else {
      set x_offset [$layer getOffsetX]
    }
    ... # -y_pitch / -y_offset 同理(getPitchY / getOffsetY)

    ifp::make_layer_tracks $layer $x_offset $x_pitch $y_offset $y_pitch
  }
}
```

要点:
- `getPitchX / getOffsetX / getPitchY / getOffsetY` 读的就是 LEF 里 `PITCH` 和 `OFFSET` 的值——这正是第 1 节那张 LEF 摘录发挥作用的地方。
- `microns_to_mfg_grid` 把用户给的微米值先对齐到制造网格,再交给 C++;LEF 读来的值本身就是 DBU,不用转。

### 6.3 C++ 核心 `makeTracks(layer, …)` —— 第 ③④⑤⑥ 步

`InitFloorplan.cc:1027`,这就是第 3 节手算的实现,一行对一步:

```cpp
void InitFloorplan::makeTracks(odb::dbTechLayer* layer,
                               int x_offset, int x_pitch,
                               int y_offset, int y_pitch)
{
  // 校验:offset 非负、pitch 为正(IFP 39~42)
  ...
  Rect die_area = block_->getDieArea();

  // 特殊处理:offset 给了 0 → 用 pitch 当起点
  // (起点在 Die 边界上,线画不出来,所以挪到第一条 pitch 处)
  if (x_offset == 0) x_offset = x_pitch;
  if (y_offset == 0) y_offset = y_pitch;

  // offset 比 Die 还大 → 整层跳过(IFP 21/22)
  if (x_offset > die_area.dx()) { ...return; }
  if (y_offset > die_area.dy()) { ...return; }

  // 取(或建)这一层的轨道网格
  auto grid = block_->findTrackGrid(layer);
  if (!grid) grid = dbTrackGrid::create(block_, layer);

  int layer_min_width = layer->getMinWidth();        // ← 半线宽检查要用

  // ── X 族(竖线)──
  int x_track_count = (die_area.dx() - x_offset) / x_pitch + 1;  // ④ 条数
  int origin_x = die_area.xMin() + x_offset;                      // ③ 起点
  if (origin_x - layer_min_width / 2 < die_area.xMin()) {         // ⑤ 左边界
    origin_x += x_pitch; x_track_count--;                         //    整体让位
  }
  int last_x = origin_x + (x_track_count - 1) * x_pitch;
  if (last_x + layer_min_width / 2 > die_area.xMax()) {           // ⑤ 右边界
    x_track_count--;                                              //    减一条
  }
  grid->addGridPatternX(origin_x, x_track_count, x_pitch);        // ⑥ 记入网格

  // ── Y 族(横线),完全对称 ──
  int y_track_count = (die_area.dy() - y_offset) / y_pitch + 1;
  int origin_y = die_area.yMin() + y_offset;
  if (origin_y - layer_min_width / 2 < die_area.yMin()) {
    origin_y += y_pitch; y_track_count--;
  }
  int last_y = origin_y + (y_track_count - 1) * y_pitch;
  if (last_y + layer_min_width / 2 > die_area.yMax()) {
    y_track_count--;
  }
  grid->addGridPatternY(origin_y, y_track_count, y_pitch);
}
```

注意第 ⑤ 步的**两种修剪手段**:
- 第一条轨道太贴左/下边界 → **整体右/上移一个 pitch**,同时**减一条**(把最后一条挤出去,维持总长度);
- 最后一条轨道太贴右/上边界 → **只减一条**。

这就是第 3 节 X 族从 7143 变 7142 的代码依据。

### 6.4 不给层的版本 `makeTracks()` —— 遍历所有层

`InitFloorplan.cc:987`。`make_tracks` 不带层参数时走到这里,把**每一层**路由金属都铺一遍:

```cpp
void InitFloorplan::makeTracks()
{
  for (auto layer : ...->getLayers()) {
    if (layer->getType() == dbTechLayerType::ROUTING
        && layer->getRoutingLevel() != 0) {
      if (layer->getFirstLastPitch() > 0) {
        makeTracksNonUniform(...);      // 特殊工艺:首尾轨道间距不同
      } else {
        const int x_pitch = layer->getPitchX();
        const int y_pitch = layer->getPitchY();
        if (x_pitch == 0 || y_pitch == 0) {
          logger_->warn(IFP, 56, "No pitch found layer {} ...");   // 层没 pitch → 跳过
          continue;
        }
        makeTracks(layer, layer->getOffsetX(), x_pitch,
                   layer->getOffsetY(), y_pitch);   // 用 LEF 默认值
      }
    }
  }
}
```

### 6.5 特殊路径:`makeTracksNonUniform`(LEF58_PITCH)

`InitFloorplan.cc:1117`。个别工艺里,同一层金属**中间区域**和**每行边缘**的轨道间距不同(行边缘要更稀),LEF 用 `LEF58_PITCH` 属性描述。此时:

- 只支持水平走线层(否则 IFP 44);
- 找到 CORE 类 site 的行高(`cell_row_height`);
- 在每一行内部,先用首尾间距 `first_last_pitch` 生成两端轨道,中间用普通 `y_pitch`,最后一根再回 `first_last_pitch`(代码 `1117-1151`)。

这是一个较少见的兼容分支,新手了解有这回事即可。

### 6.6 轨道怎么落进 DEF

`makeTracks` 往 `dbTrackGrid` 里记了三元组 (起点, 条数, 间距)。写 DEF 时,`odb` 的 `DefOut::writeTracks`(`src/odb/src/defout/defout_impl.cpp:308`)把它翻译成 `TRACKS` 语句:

```cpp
grid->getGridPatternX(i, orgX, count, step, ...);
*_out << "TRACKS X " << defdist(orgX) << " DO " << count
      << " STEP " << defdist(step) << " LAYER " << lname << " ;\n";
// Y 族同理,前缀 TRACKS Y
```

这就是第 2 节看到的 `TRACKS X 200 DO 5000 STEP 400 LAYER metal1` 的出处。X pattern 输出为 `TRACKS X`,Y pattern 输出为 `TRACKS Y`。

### 6.7 和 `make_rows` 的一个关键差异

| | `make_rows` | `make_tracks` |
| --- | --- | --- |
| 操作对象 | 放置行(单元用的格子) | 布线轨道(金属线用的网格) |
| 删旧? | **先删光旧行**再重建 | **不删**,直接填充轨道网格 |
| 默认值来源 | site 尺寸(LEF) | 层的 PITCH/OFFSET(LEF) |
| 谁来删轨道? | — | `initialize_floorplan` 建 Die 时会 `resetTracks` 清空 |

---

## 7. 常见问题(FAQ)

**Q1: 轨道和行(Row)到底啥区别?**
行管"单元放哪",由 site 的宽高决定;轨道管"金属线走哪",由金属层的 pitch 决定。一个在布局阶段用,一个在布线阶段用。`make_rows` 生成前者,`make_tracks` 生成后者。

**Q2: 为什么每层都有两族轨道(X 和 Y)?**
每层金属虽然只走一个固定方向,但轨道网格同时记录两族:走线用本层方向那一族,另一族用于辅助(比如打孔、相邻层对齐)。所以 DEF 里每个 metal 层都有 `TRACKS X` 和 `TRACKS Y` 两条。

**Q3: 为什么 `DO` 是 7142 而不是 7143?**
因为 X 族的 offset 比 Y 族大 50 DBU,算到最后一条轨道时,它的中心线离 Die 右边界不足半个线宽,画不出一条完整的金属线,所以被舍掉。看第 3 节"进阶"的手算。

**Q4: `-x_pitch` 和 `-y_pitch` 分别管什么?**
`-x_pitch` 管竖线族(X 族)的间距,`-y_pitch` 管横线族(Y 族)的间距。不要看名字想当然——"x" 指的是"x 方向上的间距",对应固定 x 的竖线族。

**Q5: 不给层名时,能带 `-x_pitch` 这类参数吗?**
不能(会报参数错误)。因为不给层名时 `make_tracks` 要遍历**所有**层,每一层的 pitch 不一样,不可能用一个参数统一。想指定 pitch 就必须指定到具体层。

**Q6: 轨道一定从 Die 左下角开始吗?**
不是。第一条轨道在 `Die起点 + offset` 处(第 3 节第一步);offset 可能不等于 0。而且如果给 `-x_offset 0`,代码会用 pitch 代替起点(第 6.3 节),避免轨道落在 Die 边界上。

**Q7: 我跑 `make_tracks` 却没看到某层有轨道,为什么?**
大概率该层的 pitch 为 0(LEF 没定义,IFP 56 警告),或者 offset 比 Die 尺寸还大(IFP 21/22,整层跳过)。查这两类警告即可。

---

## 8. 错误与警告速查

| 码 | 级别 | 含义 | 见 |
| --- | --- | --- | --- |
| IFP 10 | 错误 | 找不到这个金属层 | §6.2 |
| IFP 21/22 | 警告 | offset 比 Die 宽/高还大,整层轨道被跳过 | §6.3 |
| IFP 25 | 错误 | 指定层不是 ROUTING 类型(比如给了 CUT 层) | §6.2 |
| IFP 39/41 | 错误 | offset 为负 | §6.3 |
| IFP 40/42 | 错误 | pitch 非正 | §6.3 |
| IFP 44 | 错误 | 非水平层用了 LEF58_PITCH(不支持的组合) | §6.5 |
| IFP 45 | 错误 | 该层找不到放置行,无法算非均匀轨道 | §6.5 |
| IFP 56 | 警告 | 层没有 pitch,不生成轨道 | §6.4 |

完整清单以 `src/ifp/messages.txt` 为准。

---

## 9. 回归测试

`src/ifp/test/make_tracks1~7.{tcl,defok}` 从简到繁覆盖:

| 测试 | 覆盖点 |
| --- | --- |
| make_tracks1 | 不给层名,所有路由层用 LEF 默认值(对应第 3 节"进阶"的 7142/7143) |
| make_tracks2 | 指定层 + 显式 `-x/y_offset`、`-x/y_pitch`(第 3 节主例) |
| make_tracks3 | Die 左下角不在 (0,0),验证 origin 从 Die 起点算起 |
| make_tracks4/5/6/7 | 覆盖 offset 越界跳过、以及更复杂的组合 |

每个测试都是:跑命令 → `write_def` → 与 golden 文件 `*.defok` 逐字节比对。本文第 2、3 节用的正是 make_tracks1/2 的数据。

---

## 10. 小结

`make_tracks` 和 `make_rows` 是"给空版图补基础设施"的一对兄弟:

1. **概念上**,它回答"线往哪走"的问题——在每层金属上铺两族等距平行轨道(X 竖线族、Y 横线族),布线器把金属线中心压在轨道上。
2. **数学上**,核心是三个数:起点 `origin = Die起点 + offset`、条数 `count = (Die尺寸 - offset) / pitch + 1`,外加一个"半线宽边界修剪"。
3. **工程上**,由 Tcl(解析参数、读 LEF 默认值)→ SWIG(适配)→ C++(`makeTracks`)三层实现;不指定层时遍历所有路由层;遇到 LEF58_PITCH 的特殊层走 `makeTracksNonUniform`。

读完这一篇,再看 `initialize_floorplan`,你就明白它内部其实调用了 `makeRows` + `makeTracks` 的同一套实现——`make_rows` 和 `make_tracks` 是把那一步单独拎出来给你单独用。
