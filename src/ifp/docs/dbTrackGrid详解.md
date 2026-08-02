# dbTrackGrid 详解

> 面向 OpenROAD 新人的 dbTrackGrid 介绍文档。本文假设读者已经了解 OpenDB 的基本概念（dbBlock, dbTechLayer 等）。

## 目录

1. [概念：什么是 Track？](#1-什么是-track)
2. [dbTrackGrid 在 ODB 中的位置](#2-dbtrackgrid-在-odb-中的位置)
3. [DEF 中的 TRACKS 语法](#3-def-中的-tracks-语法)
4. [内部数据结构](#4-内部数据结构)
5. [完整 API 参考](#5-完整-api-参考)
6. [Pattern（模式）的概念](#6-多-pattern-概念)
7. [数据流转：从 DEF 到 TrackGrid 再到写出](#7-数据流转)
8. [使用示例](#8-使用示例)
9. [上下游模块如何消费 TrackGrid](#9-上下游模块如何消费-trackgrid)
10. [与 dbGCellGrid 的区别](#10-与-dbgcellgrid-的区别)
11. [常见误区](#11-常见误区)

---

## 1. 什么是 Track？

在物理设计中，**Track（布线轨道）** 是芯片上预定义的布线网格线。每条金属线必须放置在 track 上，不能随意摆放。这样做的好处是：

- **保证可制造性（Manufacturability）**：线宽和间距符合工艺要求
- **简化布线算法**：布线器只在离散的网格线上搜索路径，而不是连续空间
- **保证对齐**：同层或不同层的走线对齐，避免 DRC 违规

Track 有两个方向：
- **X 方向 track**：垂直走线位置（走线方向为垂直）
- **Y 方向 track**：水平走线位置（走线方向为水平）

每个技术层（Metal1, Metal2, ...）都有各自的 track 定义。通常：
- Metal1 走水平方向 → 定义 Y 方向的 track
- Metal2 走垂直方向 → 定义 X 方向的 track
- 以此类推，相邻层交替

---

## 2. dbTrackGrid 在 ODB 中的位置

```
odb::dbBlock
  └── dbSet<dbTrackGrid> getTrackGrids()    // 获取该 block 的所有 track grids
  └── dbTrackGrid* findTrackGrid(dbTechLayer*)  // 按层查找 track grid

odb::dbTrackGrid : public dbObject
  └── 每个 dbTrackGrid 关联一个 dbTechLayer
  └── 包含该层上所有的 X/Y 轨道模式
```

**核心关系**：
- 一个 `dbBlock` 可以包含**多个** `dbTrackGrid`（每个 routing layer 最多一个）
- 每个 `dbTrackGrid` 绑定**一个** `dbTechLayer`
- 每个 `dbTrackGrid` 可以有**多个** X pattern 和**多个** Y pattern

类层次结构：
```
_dbObject              (内部基类，存储层，OID 等)
  └── _dbTrackGrid     (内部实现类，包含所有数据成员)
        └── dbTrackGrid (公开 API 类，提供面向用户的方法)
```

---

## 3. DEF 中的 TRACKS 语法

Track 信息来自 DEF 文件的 `TRACKS` 段落。语法如下：

```
TRACKS X <start> DO <count> STEP <step>
    [ MASK <mask_num> [ SAMEMASK ] ]
    LAYER <layer_name> ;

TRACKS Y <start> DO <count> STEP <step>
    [ MASK <mask_num> [ SAMEMASK ] ]
    LAYER <layer_name> ;
```

**参数解释**：

| 参数 | 含义 | 示例 |
|------|------|------|
| `X` / `Y` | track 方向 | `TRACKS X` 表示定义 X 方向 track |
| `<start>` | 第一条 track 的起始坐标（DBU 单位） | `480` |
| `DO <count>` | 该组 track 的数量 | `DO 251` |
| `STEP <step>` | 两条相邻 track 之间的间距（pitch） | `STEP 480` |
| `MASK <n>` | 光刻掩模号（多图案工艺） | `MASK 2` |
| `SAMEMASK` | 所有 track 使用同一掩模（不交替） | 省略则交替分配 |
| `LAYER <name>` | 该组 track 所属的金属层 | `LAYER metal1` |

**实际 DEF 示例**：

```
TRACKS Y 720 DO 300 STEP 480 LAYER metal1 ;
TRACKS X 720 DO 300 STEP 480 LAYER metal1 ;
TRACKS Y 960 DO 400 STEP 720 LAYER metal2 ;
TRACKS X 960 DO 400 STEP 720 LAYER metal2 ;
```

这里：
- metal1 定义了 Y track：从 720 开始，共 300 条，间距 480
- metal1 也定义了 X track：从 720 开始，共 300 条，间距 480
- metal2 同理，间距变为 720

> **注意**：DEF 文件允许多个 `TRACKS` 语句指向同一个层，每个语句对应一个 pattern。

---

## 4. 内部数据结构

### 4.1 内部类 `_dbTrackGrid`

定义文件：`src/odb/src/db/dbTrackGrid.h`

```cpp
class _dbTrackGrid : public _dbObject
{
public:
    // --- 持久化数据（存入数据库文件） ---
    dbId<_dbTechLayer> layer_;          // 关联的技术层
    dbVector<int> x_origin_;            // 每个 X pattern 的起始坐标
    dbVector<int> x_count_;             // 每个 X pattern 的 track 数量
    dbVector<int> x_step_;              // 每个 X pattern 的 track 间距
    dbVector<int> y_origin_;            // 每个 Y pattern 的起始坐标
    dbVector<int> y_count_;             // 每个 Y pattern 的 track 数量
    dbVector<int> y_step_;              // 每个 Y pattern 的 track 间距
    dbVector<int> first_mask_;          // 每个 pattern 的首个掩模号（多图案工艺）
    dbVector<bool> samemask_;           // 每个 pattern 是否所有 track 同掩模
    dbId<_dbTechLayer> next_grid_;      // 链表指针（下一层的 track grid）

    // --- 瞬态数据（不持久化，运行时缓存） ---
    std::vector<int> grid_x_;           // 展开后的所有 X 坐标（懒加载）
    std::vector<int> grid_y_;           // 展开后的所有 Y 坐标（懒加载）
};
```

### 4.2 字段详解

**`x_origin_`, `x_count_`, `x_step_`**（Y 方向同理）

这三个数组一一对应，第 `i` 个 pattern 描述了一组等间距的 track：
- `origin[i]`：该 pattern 第一条 track 的坐标
- `count[i]`：该 pattern 包含多少条 track
- `step[i]`：该 pattern 中 track 之间的步长

track 位置 = `origin[i] + j * step[i]`，其中 `j = 0, 1, ..., count[i]-1`

**`first_mask_` 和 `samemask_`**

用于多图案（Multi-Patterning）工艺。现代工艺中，间距太近的金属线无法用同一次光刻完成，需要分解到多个掩模。
- `first_mask_`：该 pattern 第一条 track 的掩模号
- `samemask_`：`true` = 所有 track 用同一个掩模；`false` = 交替使用不同掩模

**`grid_x_` 和 `grid_y_`**（瞬态缓存）

将多个 pattern 展开、合并、排序、去重后的所有 track 坐标。首次调用 `getGridX()` / `getGridY()` 时懒加载计算。修改 pattern（调用 `addGridPatternX/Y`）后会清空此缓存。

### 4.3 相等性比较

两个 `_dbTrackGrid` 相等的条件是所有持久化字段都相等（`operator==`）。排序比较（`operator<`）仅按 `layer_` 排序（遍历 block 的所有 track grid 时保证按层的顺序）。

---

## 5. 完整 API 参考

定义文件：`src/odb/include/odb/db.h`（行 3878-3975）

### 5.1 创建与销毁

```cpp
// 为指定的 block 和 layer 创建一个空的 TrackGrid
// 如果该 layer 已经有 TrackGrid，返回 nullptr
static dbTrackGrid* create(dbBlock* block, dbTechLayer* layer);

// 通过数据库 ID 获取指针
static dbTrackGrid* getTrackGrid(dbBlock* block, uint32_t oid);

// 销毁一个 TrackGrid
static void destroy(dbTrackGrid* grid);
```

### 5.2 查询所属关系

```cpp
// 获取该 grid 所属的技术层
dbTechLayer* getTechLayer();

// 获取该 grid 所属的 block
dbBlock* getBlock();
```

### 5.3 添加 Pattern

```cpp
// 添加一个 X 方向的 grid pattern
void addGridPatternX(int origin_x,
                     int line_count,
                     int step,
                     int first_mask = 0,
                     bool samemask = false);

// 添加一个 Y 方向的 grid pattern
void addGridPatternY(int origin_y,
                     int line_count,
                     int step,
                     int first_mask = 0,
                     bool samemask = false);
```

### 5.4 查询 Pattern 数量

```cpp
int getNumGridPatternsX();   // X 方向 pattern 的个数
int getNumGridPatternsY();   // Y 方向 pattern 的个数
```

### 5.5 读取单个 Pattern

不带掩模信息：
```cpp
void getGridPatternX(int i, int& origin_x, int& line_count, int& step);
void getGridPatternY(int i, int& origin_y, int& line_count, int& step);
```

带掩模信息：
```cpp
void getGridPatternX(int i,
                     int& origin_x,
                     int& line_count,
                     int& step,
                     int& first_mask,
                     bool& samemask);
void getGridPatternY(int i,
                     int& origin_y,
                     int& line_count,
                     int& step,
                     int& first_mask,
                     bool& samemask);
```

> **C++ 使用提示**：你需要先声明 `int origin, count, step;` 再传入引用。

### 5.6 获取展开后的所有 Track 坐标

```cpp
// 返回引用（零拷贝，但内容可能被后续修改清空）
const std::vector<int>& getGridX();
const std::vector<int>& getGridY();

// 拷贝到一个外部 vector（安全，需要在外部分配好 vector）
void getGridX(std::vector<int>& x_grid);
void getGridY(std::vector<int>& y_grid);
```

展开过程：遍历所有 pattern，对每个 pattern 生成 `origin + j*step`（j = 0 到 count-1），最后排序去重。

### 5.7 获取平均 Track 间距

```cpp
void getAverageTrackSpacing(int& track_step,    // [out] 平均间距
                            int& track_init,    // [out] 起始坐标
                            int& num_tracks);   // [out] track 总数
```

这个方法会根据层方向自动选择用 X 还是 Y 的 track 数据：
- `HORIZONTAL` 层 → 读取 Y pattern（因为水平走线，track 是 Y 方向的线）
- `VERTICAL` 层 → 读取 X pattern（因为垂直走线，track 是 X 方向的线）

如果只有一个 pattern，直接返回该 pattern 的 step/origin/count。如果有多个 pattern（间距不一致），则返回平均值。

---

## 6. 多 Pattern 概念

### 为什么一个层需要多个 Pattern？

在某些工艺或设计中，同一层的 track 间距可能**不均匀**。例如：

```
TRACKS X 100 DO 50  STEP 200 LAYER metal3 ;    // Pattern 0: 间距 200
TRACKS X 120 DO 100 STEP 100 LAYER metal3 ;    // Pattern 1: 间距 100
```

这时 metal3 层有两个 X pattern。展开后：
- Pattern 0 产生：100, 300, 500, 700, ...（粗间距区域）
- Pattern 1 产生：120, 220, 320, 420, ...（细间距区域）
- 合并排序后得到完整的 track 集合

### 数据模型示意图

```
dbTrackGrid (for metal3)
│
├── x_origin_[0] = 100    x_count_[0] = 50    x_step_[0] = 200   ← Pattern 0
├── x_origin_[1] = 120    x_count_[1] = 100   x_step_[1] = 100   ← Pattern 1
│
└── grid_x_ (transient cache)
    = [100, 120, 220, 300, 320, 420, 500, ...]  ← 展开、排序、去重后的完整集合
```

---

## 7. 数据流转

TrackGrid 有**三个**主要创建来源：

| 来源 | 说明 | 优先级 |
|------|------|--------|
| DEF 文件 `TRACKS` 段落 | 外部工具（Innovus/ICC2）导出的 track 定义 | 高（直接读取已有数据） |
| `make_tracks` 命令 | OpenROAD 内部根据 LEF pitch/offset 自动生成 | 中（没有 DEF tracks 时使用） |
| 手动 API 调用 | 通过 C++/Python/Tcl API 创建 | 低（脚本自定义场景） |

### 7.1 来源一：make_tracks 命令（IFP 模块）

当 DEF 中没有 TRACKS 段落时，通常使用 `make_tracks` 命令按 LEF 中定义的 pitch 信息生成 track。

**文件**：`src/ifp/src/InitFloorplan.cc`（`makeTracks()`、`makeTracksNonUniform()`、`resetTracks()`）

**计算逻辑**（伪代码）：
```
for each routing layer:
    pitch  = layer->getPitchX()  (或 getPitchY())    ← 从 LEF PITCH 读取
    offset = layer->getOffsetX() (或 getOffsetY())    ← 从 LEF OFFSET 读取
    origin = dieArea.xMin() + offset                   ← 起始位置
    count  = (dieArea.dx() - offset) / pitch + 1       ← track 数量
    grid->addGridPatternX(origin, count, pitch)        ← 添加到 grid
```

**Tcl 用法**：
```tcl
make_tracks metal1 -x_pitch 0.48 -y_pitch 0.48
make_tracks                         ;# 不指定层则为所有 routing layer 生成
make_tracks -clean                  ;# 先清除已有 tracks 再生成
```

**相关命令**：
- `resetTracks()` → 清除所有 TrackGrid（`dbTrackGrid::destroy`）

### 7.2 来源二：读取 DEF（输入）

```
DEF 文件
    │
    ▼
definTracks::tracksBegin()    ← 解析 TRACKS 语句的方向、起始、数量、步长、掩模
    │
    ▼
definTracks::tracksLayer()    ← 解析 LAYER 名称
    │
    ├── block->findTrackGrid(layer)   ← 查找是否已存在
    │   └── 不存在 → dbTrackGrid::create(block, layer)   ← 创建新的
    │
    └── grid->addGridPatternX() 或 addGridPatternY()   ← 添加 pattern
```

关键代码（`src/odb/src/defin/definTracks.cpp`）：

```cpp
void definTracks::tracksLayer(const char* layer_name)
{
    dbTechLayer* layer = _tech->findLayer(layer_name);
    if (layer == nullptr) {
        _logger->warn(utl::ODB, 165, "undefined layer ({}) referenced", layer_name);
        ++_errors;
        return;
    }

    dbTrackGrid* grid = _block->findTrackGrid(layer);
    if (grid == nullptr) {
        grid = dbTrackGrid::create(_block, layer);     // 不存在则创建
    }

    if (_track._dir == DEF_X) {
        grid->addGridPatternX(_track._orig, _track._count,
                              _track._step, _track._first_mask, _track._samemask);
    } else {
        grid->addGridPatternY(_track._orig, _track._count,
                              _track._step, _track._first_mask, _track._samemask);
    }
}
```

### 7.3 来源三：写出 DEF（输出）

`src/odb/src/defout/defout_impl.cpp` 中遍历所有的 TrackGrid 及其 pattern：

```cpp
for (dbTrackGrid* grid : block->getTrackGrids()) {
    // 输出所有 X pattern
    for (int i = 0; i < grid->getNumGridPatternsX(); ++i) {
        int orgX, count, step, firstmask;
        bool samemask;
        grid->getGridPatternX(i, orgX, count, step, firstmask, samemask);
        // 写出: TRACKS X orgX DO count STEP step MASK firstmask [SAMEMASK]
        //       LAYER layerName ;
    }
    // 输出所有 Y pattern
    for (int i = 0; i < grid->getNumGridPatternsY(); ++i) {
        // 类似 ...
    }
}
```

### 7.3 生命周期管理

```
创建: dbTrackGrid::create(block, layer) → block->track_grid_tbl_->create()
          ↓
使用: block->findTrackGrid(layer) 或 block->getTrackGrids()
          ↓
销毁: dbTrackGrid::destroy(grid) → 清理属性 → block->track_grid_tbl_->destroy()
```

所有 TrackGrid 存储在 `_dbBlock::track_grid_tbl_`（`dbTable<_dbTrackGrid>`）中，随 block 一起持久化到 `.odb` 数据库文件。

---

## 8. 使用示例

### 8.1 C++ 示例

```cpp
#include "odb/db.h"

void example(odb::dbDatabase* db)
{
    odb::dbChip* chip = db->getChip();
    odb::dbBlock* block = chip->getBlock();
    odb::dbTech* tech = db->getTech();

    // --- 创建 TrackGrid ---
    odb::dbTechLayer* metal1 = tech->findLayer("metal1");
    odb::dbTrackGrid* grid = odb::dbTrackGrid::create(block, metal1);

    // --- 添加 pattern ---
    // Y 方向: 从 720 开始，300 条 track，间距 480
    grid->addGridPatternY(720, 300, 480);
    // X 方向: 从 720 开始，300 条 track，间距 480
    grid->addGridPatternX(720, 300, 480);

    // --- 查询 ---
    // 获取展开后的所有 Y 坐标
    const std::vector<int>& y_tracks = grid->getGridY();
    std::cout << "Total Y tracks: " << y_tracks.size() << std::endl;
    // 输出: 300 (720, 1200, 1680, ...)

    // 读取某个 pattern 的参数
    for (int i = 0; i < grid->getNumGridPatternsY(); i++) {
        int origin, count, step;
        grid->getGridPatternY(i, origin, count, step);
        std::cout << "Pattern " << i << ": origin=" << origin
                  << " count=" << count << " step=" << step << std::endl;
    }

    // --- 获取平均间距 ---
    int track_step, track_init, num_tracks;
    grid->getAverageTrackSpacing(track_step, track_init, num_tracks);
    // 如果 metal1 方向是 HORIZONTAL，则返回 Y track 的信息

    // --- 遍历所有层的 TrackGrid ---
    for (odb::dbTrackGrid* g : block->getTrackGrids()) {
        odb::dbTechLayer* layer = g->getTechLayer();
        std::cout << "Layer: " << layer->getName() << std::endl;
        std::cout << "  X patterns: " << g->getNumGridPatternsX() << std::endl;
        std::cout << "  Y patterns: " << g->getNumGridPatternsY() << std::endl;
    }
}
```

### 8.2 Python 示例

```python
import odb

# 假设 db 已经加载了 DEF 文件
block = db.getChip().getBlock()

# --- 遍历所有 TrackGrid ---
for track_grid in block.getTrackGrids():
    layer = track_grid.getTechLayer()
    print(f"Layer: {layer.getName()}")
    print(f"  Direction: {layer.getDirection()}")

    # 获取 X 方向的 pattern 数量
    for i in range(track_grid.getNumGridPatternsX()):
        origin, count, step = track_grid.getGridPatternX(i)
        print(f"  X Pattern {i}: origin={origin}, count={count}, step={step}")

    # 获取展开后的所有 Y 坐标
    y_grid = track_grid.getGridY()
    print(f"  Total Y tracks: {len(y_grid)}")
    if len(y_grid) > 0:
        print(f"  First: {y_grid[0]}, Last: {y_grid[-1]}")
```

### 8.3 Tcl 示例

```tcl
# 加载 DEF
read_def design.def

# 获取 block
set block [[[ord::get_db] getChip] getBlock]

# 遍历 track grids
foreach grid [$block getTrackGrids] {
    set layer [$grid getTechLayer]
    puts "Layer: [$layer getName]"
    puts "  Num X patterns: [$grid getNumGridPatternsX]"
    puts "  Num Y patterns: [$grid getNumGridPatternsY]"
}

# 检查某个 layer 是否有 track（使用便捷函数）
if {[db_layer_has_tracks metal2]} {
    puts "metal2 has tracks defined"
}
```

> **注意**：`db_layer_has_tracks` 是 OpenROAD 在 `src/OpenRoad.i` 中定义的 Tcl 辅助函数，用于快速检查某层是否有 track 定义。

### 8.4 DRT 中 TrackGrid 的转换

在详细布线（TritonRoute）中，`dbTrackGrid` 会被转换为内部表示 `frTrackPattern`：

```cpp
// src/drt/src/io/io.cpp - setTracks()
void io::Parser::setTracks(odb::dbBlock* block)
{
    for (odb::dbTrackGrid* track_grid : block->getTrackGrids()) {
        odb::dbTechLayer* layer = track_grid->getTechLayer();
        if (layer->getType() != odb::dbTechLayerType::ROUTING) continue;

        // 读取 X pattern 并转为 frTrackPattern
        for (int i = 0; i < track_grid->getNumGridPatternsX(); i++) {
            int origin, count, step;
            track_grid->getGridPatternX(i, origin, count, step);
            // 创建 frCoord 格式的 track pattern
        }
        // Y 方向同理 ...
    }
}
```

---

## 9. 上下游模块如何消费 TrackGrid

TrackGrid 是 OpenROAD 中**被广泛消费**的基础数据。以下是各模块的使用方式：

### PPL（Pin/IO 放置）— `src/ppl/src/IOPlacer.cpp`

IO 引脚必须对齐到 track 上：

```cpp
odb::dbTrackGrid* track_grid = getBlock()->findTrackGrid(layer);
if (track_grid->getNumGridPatternsY() > 0) {
    int init_track, num_track, min_spacing;
    track_grid->getGridPatternY(0, init_track, num_track, min_spacing);
    // 使用 track 位置来放置 I/O pins
}
```

### MPL（Macro 放置）— `src/mpl/src/snapper.cpp`

Macro 放置后需要"吸附"（snap）到最近的 track：

```cpp
// 按 track_grid 对 pins 分组
TrackGridToPinListMap track_grid_to_pin_list;

// 获取所有 track 位置
void Snapper::getTrackGridPattern(odb::dbTrackGrid* track_grid, ...)
{
    // 根据 layer 方向读取 X 或 Y 的 pattern
    track_grid->getGridPatternX(pattern_idx, origin, count, step);
    // 或
    track_grid->getGridPatternY(pattern_idx, origin, count, step);
}
```

### GRT（全局布线）— `src/grt/src/GlobalRouter.cpp`

全局布线需要 track 信息来确定布线容量和拥塞：

```cpp
// 使用 layer 的 direction + track pitch 构建布线图
// 通过 block->findTrackGrid(layer) 获取 pitch 信息
```

### DRT（详细布线）— `src/drt/src/TritonRoute.cpp`

详细布线是最核心的 track 消费者，每条导线必须落在 track 上。

### PDN（电源网络）— `src/pdn/src/techlayer.cpp`

电源网络布线时对齐到 track。

### GUI — `src/gui/src/renderThread.cpp`

GUI 渲染时显示 track 网格线（作为背景参考线）。

### RCX（寄生提取）— `src/rcx/src/extFlow.cpp`

提取寄生参数时，需要 track 信息来判断导线的位置。

---

## 10. 与 dbGCellGrid 的区别

OpenDB 中有两个看起来相似的 Grid 概念，但用途完全不同：

| 特性 | `dbTrackGrid` | `dbGCellGrid` |
|------|---------------|---------------|
| **用途** | 定义布线轨道（routing tracks） | 定义全局布线单元（Global Routing Cell） |
| **粒度** | 细粒度（与金属 pitch 同数量级） | 粗粒度（通常几十条 track 组成一个 GCell） |
| **数据来源** | DEF 的 `TRACKS` 段落 | DEF 的 `GCELLGRID` 段落 |
| **使用者** | 详细布线、引脚放置、所有布线阶段 | 全局布线的拥塞分析 |
| **关联层** | 每个 routing layer 一个 | 所有层共享同一个网格 |
| **方向** | 按层有方向性（HORIZONTAL/VERTICAL） | 只有 X/Y 方向的网格线 |
| **pattern** | 支持多 pattern | 也支持多 pattern |

**简单理解**：
- **TrackGrid** = "金属线可以画在哪些线上"（更细）
- **GCellGrid** = "芯片被划分成哪些方格来做全局布线规划"（更粗）

---

## 11. 常见误区

### 误区 1：HORIZONTAL 层 = 看 X track

**错误**。层的 `direction` 指的是**走线方向**，不是 track 线方向。

- `HORIZONTAL` 层 → 金属线水平走 → track 是**水平线** → 需要看 **Y** 坐标
- `VERTICAL` 层 → 金属线垂直走 → track 是**垂直线** → 需要看 **X** 坐标

验证方式：`getAverageTrackSpacing()` 的源码逻辑正是如此：
```cpp
if (layer->getDirection() == HORIZONTAL) {
    // 水平走线 → 读 Y grid
    track_grid->getAverageTrackPattern(false, ...);
} else if (layer->getDirection() == VERTICAL) {
    // 垂直走线 → 读 X grid
    track_grid->getAverageTrackPattern(true, ...);
}
```

### 误区 2：一个 layer 只能有一个 TrackGrid

**正确**。每个 layer 只能创建一个 `dbTrackGrid`。如果尝试重复创建，`create()` 返回 `nullptr`：
```cpp
dbTrackGrid* dbTrackGrid::create(dbBlock* block_, dbTechLayer* layer_)
{
    if (block_->findTrackGrid(layer_)) {
        return nullptr;   // 已存在，不创建
    }
    // ...
}
```

但一个 TrackGrid 可以包含**多个 pattern**（通过多次调用 `addGridPatternX/Y`）。

### 误区 3：getGridX() 和 getGridY() 是实时计算的

**错误**。它们是**懒加载缓存**的。首次调用时展开所有 pattern 并排序去重，之后直接返回缓存。添加新 pattern 时缓存被清空（`grid_x_.clear()`）。

这意味着：
- 第一次调用较慢（需要展开所有 pattern）
- 后续调用是 O(1) 返回引用
- 修改 pattern 后下次调用会重新计算

### 误区 4：Track pitch 一定是均匀的

**不一定**。多 pattern 场景下，不同 pattern 的 `step` 可以不同。`getAverageTrackSpacing()` 只返回平均值。

如果一个层的 pattern 间距差异很大，建议逐个读取 pattern 而不要依赖平均值。

### 误区 5：所有 routing layer 一定有 TrackGrid

**不一定**。某些流程可能不会为所有 routing layer 定义 track。在使用前应当检查：
```cpp
odb::dbTrackGrid* grid = block->findTrackGrid(layer);
if (grid == nullptr) {
    // 该层没有 TrackGrid，可能需要在 DEF 中添加
}
```

---

## 附录：源码文件索引

| 文件 | 说明 |
|------|------|
| `src/odb/src/db/dbTrackGrid.h` | 内部实现类 `_dbTrackGrid` 定义 |
| `src/odb/src/db/dbTrackGrid.cpp` | 公开 API `dbTrackGrid` 实现 |
| `src/odb/include/odb/db.h` (行 3875-3975) | 公开 API 声明 |
| `src/odb/include/odb/dbObject.h` | 对象类型枚举 `dbTrackGridObj = 0xB` |
| `src/odb/src/db/dbBlock.h` | `_dbBlock` 中的 `track_grid_tbl_` 声明 |
| `src/odb/src/db/dbBlock.cpp` | `findTrackGrid()`, `getTrackGrids()` 实现 |
| `src/odb/src/db/dbDatabase.h` | Schema 版本：`kSchemaTrackMask = 99` |
| `src/odb/src/defin/definTracks.h` | DEF TRACKS 解析器声明 |
| `src/odb/src/defin/definTracks.cpp` | DEF TRACKS 解析器实现 |
| `src/odb/src/defout/defout_impl.cpp` | DEF 写出 TrackGrid |
| `src/ifp/src/InitFloorplan.cc` | `make_tracks` 命令实现（`makeTracks()`, `resetTracks()`） |
| `src/ifp/src/InitFloorplan.tcl` | `make_tracks` Tcl 命令 |
| `src/drt/src/io/io.cpp` | `setTracks()` — TrackGrid → frTrackPattern 转换 |
| `src/OpenRoad.i` | `db_layer_has_tracks()` Tcl 辅助函数 |
| `src/odb/src/swig/common/dbtypes_common.i` | SWIG 输出参数绑定 |
| `src/odb/src/swig/common/containers.i` | SWIG 容器绑定（`dbSet<dbTrackGrid>`） |
| `src/odb/src/swig/python/dbtypes.i` | Python SWIG 类型映射 |
| `src/odb/test/check_routing_tracks.tcl` | Track 验证测试（确保所有 routing layer 有 track） |
| `src/odb/test/tcl/18-check_routing_tracks.tcl` | Track 验证测试（旧路径） |
| `src/odb/messages.txt` | 错误消息定义（代码 358, 414-418） |

### 相关错误消息

| 代码 | 级别 | 消息 | 说明 |
|------|------|------|------|
| 358 | WARN | `Track layer {} not found for LEF58_TYPE PINLAYER {}` | pin layer 对应的 track 层未定义 |
| 414 | ERROR | `Horizontal tracks for layer {} not found.` | HORIZONTAL 层缺少 Y track |
| 415 | ERROR | `Vertical tracks for layer {} not found.` | VERTICAL 层缺少 X track |
| 416 | ERROR | `Layer {} has invalid direction.` | 层方向既不是 HORIZONTAL 也不是 VERTICAL |
| 418 | ERROR | `Layer is empty.` | getTechLayer() 返回了 nullptr |

## 附录：相关概念快速索引

- `dbTechLayer::getDirection()` — 获取层的走线方向（`HORIZONTAL` / `VERTICAL`）
- `dbTechLayer::pitch_x_` / `pitch_y_` — LEF 文件中定义的层 pitch（与 TrackGrid 的 step 通常一致）
- `dbTechLayer::right_way_on_grid_only` — 标记该层是否严格要求走线在 track 上
- `dbBlock::findTrackGrid()` — 按层查找 TrackGrid
- `dbBlock::getTrackGrids()` — 获取所有 TrackGrid 的迭代集合
- `dbGCellGrid` — 全局布线网格（别搞混了！）
