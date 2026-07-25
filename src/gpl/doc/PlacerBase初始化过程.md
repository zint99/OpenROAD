# PlacerBase 初始化过程详解

> 模块：GPL (Global Placement)
> 源文件：`src/gpl/src/placerBase.h`, `src/gpl/src/placerBase.cpp`

## 概述

`PlacerBase` 是全局布局 (GPL) 的核心数据结构之一，封装布局问题中按区域 (Region/Group) 组织的实例、面积统计和不可用站点信息。其初始化依赖于 `PlacerBaseCommon`（全局共享数据），采用**两层结构**：

```
                                     Replace 入口
                                          │
                    ┌─────────────────────┴─────────────────────┐
                    │  1. PlacerBaseCommon (一次, 全局共享)      │
                    │     从 ODB 加载所有实例/引脚/线网           │
                    └─────────────────┬──────────────────────────┘
                                      │
                    ┌─────────────────┴──────────────────────────┐
                    │  2. PlacerBase (每个 Region/Group 一个)     │
                    │     从 PBC 过滤实例 → 分类 → 虚拟实例填充   │
                    └─────────────────────────────────────────────┘
```

---

## 一、`PlacerBaseCommon` 简要说明

`PlacerBaseCommon` 是全局共享的数据容器，由 `Replace` 在首次布局时创建一次，所有 `PlacerBase` 共享。其主要职责：

1. **加载所有实例**：遍历 ODB 中所有非 PAD 类型的 `dbInst`，构造 `Instance` 对象，应用 padLeft/padRight 偏移，识别 Macro，并对固定实例执行 snapOutward 吸附。
2. **引脚密度面积调整**：统计信号引脚密度，对高引脚密度可移动单元按比例放大面积（限制在 [0.95, 1.2]）。
3. **加载线网**：仅处理 SIGNAL/CLOCK 类型线网，构造 `Net` 对象并计算包围盒。
4. **加载引脚**：为每个 ITerm 和 BTerm 构造 `Pin` 对象，计算全局坐标（ITerm 需根据实例朝向旋转）。
5. **构建映射表**：`instMap_`（dbInst → Instance*）、`pinMap_`（dbITerm/dbBTerm → Pin*）、`netMap_`（dbNet → Net*）。
6. **交叉链接**：实例→引脚、线网→引脚双向绑定。

> `PlacerBaseCommon::init()` 的详细过程请参考 `PlacerBaseCommon` 相关文档。

---

## 二、`PlacerBase::init()` — 主流程

构造函数原型：

```cpp
PlacerBase(odb::dbDatabase* db,
           std::shared_ptr<PlacerBaseCommon> pbCommon,
           utl::Logger* log,
           bool check_density,
           odb::dbGroup* group = nullptr);
```

调用入口（`Replace::initNesterovPlace` / `Replace::doInitialPlace`）：

```cpp
// 1. 顶层 (top-level) 区域
pbVec_.push_back(std::make_shared<PlacerBase>(db_, pbc_, log_, check_density));

// 2. 每个物理 Region 中的 Group
for (auto pd : db_->getChip()->getBlock()->getRegions()) {
  for (auto group : pd->getGroups()) {
    pbVec_.push_back(
        std::make_shared<PlacerBase>(db_, pbc_, log_, check_density, group));
  }
}
```

`init(check_density)` 内部执行四个阶段：

```
init(check_density)
  ├── 1. 区域边界定义     region_bbox_ / region_area_
  ├── 2. 实例过滤与分类   placeInsts_ / fixedInsts_ / area 统计
  ├── 3. 虚拟实例填充     initInstsForUnusableSites()
  └── 4. 信息打印与校验   printInfo(check_density)
```

---

### 阶段 1：区域边界定义

```cpp
if (group_ != nullptr) {
  auto boundaries = group_->getRegion()->getBoundaries();
  if (!boundaries.empty()) {
    // 有显式边界 → 取所有边界的并集
    region_bbox_.mergeInit();
    for (auto boundary : boundaries)
      region_bbox_.merge(boundary->getBox());
    region_area_ = region_bbox_.area();
  } else {
    // 无显式边界 → 使用整个 Core 区域
    region_bbox_ = odb::Rect(coreLx, coreLy, coreUx, coreUy);
    region_area_ = die_.coreArea();
  }
} else {
  // 顶层区域 → 使用整个 Core 区域
  region_bbox_ = odb::Rect(coreLx, coreLy, coreUx, coreUy);
  region_area_ = die_.coreArea();
}
```

此阶段同时从 `pbCommon_` 拷贝 `Die`（含 die 与 core 包围盒）和 `siteSizeX_`/`siteSizeY_`。

---

### 阶段 2：实例过滤与分类

从 `PlacerBaseCommon` 共享的所有实例中，筛选属于当前区域的实例并进行分类。

**实例区域归属逻辑**：

| 场景 | 保留条件 |
|------|---------|
| 顶层 (`group_ == nullptr`) | 无 Group，或 Group 类型为 `VISUAL_DEBUG` |
| 区域 (`group_ != nullptr`) | Group 非空、与当前 group 一致、且非 `VISUAL_DEBUG` |

**分类规则**：

```
对每个通过筛选的实例:
  │
  ├─ isFixed() && isCoreAreaOverlap(die_, inst):
  │   ├─→ fixedInsts_       (固定实例)
  │   ├─→ nonPlaceInsts_    (非可放置集合)
  │   └─→ nonPlaceInstsArea_ += 重叠面积
  │
  └─ 可移动 (isPlaceInstance):
      ├─→ placeInsts_       (可放置集合)
      ├─→ placeInstsArea_   += 实例面积
      ├─→ dy > 6 × siteSizeY → macroInstsArea_ += 面积
      └─→ 否则 → stdInstsArea_ += 面积
```

**关键判断函数**：

- `isCoreAreaOverlap(die, inst)`：取实例与 Core 包围盒的重叠矩形，若 `rectLx < rectUx && rectLy < rectUy` 则重叠。
- `getOverlapWithCoreArea(die, inst)`：计算重叠面积（用于 `nonPlaceInstsArea_`）。
- 高度阈值 `6 × siteSizeY` 是 Macro 判据（与 `PlacerBaseCommon` 中相同，此处重新统计区域内的值）。

> 所有通过筛选的实例最终均加入 `pb_insts_` 向量，构成该区域实例的完整列表。

---

### 阶段 3：虚拟实例生成 — `initInstsForUnusableSites()`

将 Core 区域内**不可放置的站点**（碎片行、Blockage、Macro Halo、其他 Region 占用的站点）填充为虚拟 (Dummy) 实例，确保密度网格覆盖完整。

#### 3.1 站点网格初始化

```
siteCountX = coreDx / siteSizeX
siteCountY = coreDy / siteSizeY
siteGrid[siteCountX × siteCountY], 全部初始化为 Row（可放置）
```

枚举 `SiteInfo` 三种状态：

| 标记 | 含义 |
|------|------|
| `Row` | 正常可放置站点 |
| `Blocked` | 不可用，需生成 Dummy 实例 |
| `FixedInst` | 被固定实例占用的站点 |

#### 3.2 站点标记流程

**(a) 顶层区域的非 Row 区域标记为 Blocked**

仅当 `group_ == nullptr`（顶层区域）时执行：

```cpp
siteGrid 全标记为 Blocked
遍历所有 Row: 标记被 Row 覆盖的站点为 Row
遍历所有 Group Region: 将被 Region 覆盖的站点重新标记为 Blocked
```

> 区域模式下 `siteGrid` 保持初始的 `Row` 状态，不会将非 Row 区域标记为 Blocked，因为区域本身的设计意图就是在该子区域内自由放置。

**(b) Blockage（布线阻挡）处理**

```cpp
for (dbBlockage* blockage : block->getBlockages()) {
  if (inst && !inst->isFixed())
    error("Blockages associated with moveable instances are unsupported")
  // 按 maxDensity 计算阻挡比例，部分标记为 Blocked
  filler_density = (100 - blockage->getMaxDensity()) / 100
  while (filled / cells <= filler_density)
    siteGrid[j * siteCountX + i] = Blocked
}
```

**(c) 固定实例占用标记**

```cpp
for (auto& inst : pbCommon_->getInsts()) {
  // 同阶段 2 的区域归属过滤
  if (inst->isMacro() && inst->dbInst()->getHalo() != nullptr) {
    // Macro Halo → Blocked
    Rect halo = inst->dbInst()->getTransformedHalo();
    for (每个被 Halo 覆盖的站点)
      siteGrid[...] = Blocked
  }
  // 实例本身占用的站点 → FixedInst
  for (每个被实例覆盖的站点)
    siteGrid[...] = FixedInst
}
```

#### 3.3 Dummy 实例生成

```cpp
for (int j = 0; j < siteCountY; j++) {
  for (int i = 0; i < siteCountX; i++) {
    if (siteGrid[j * siteCountX + i] == Blocked) {
      startX = i;
      while (i < siteCountX && siteGrid[...] == Blocked) i++;
      endX = i;
      // 构造 Dummy Instance (dbInst == nullptr)
      instStor_.emplace_back(
        coreLx + startX * siteSizeX,
        coreLy + j * siteSizeY,
        coreLx + endX * siteSizeX,
        coreLy + (j + 1) * siteSizeY);
    }
  }
}
```

**Dummy 实例的后续处理**（在 `init()` 中紧接调用）：

```cpp
for (auto& inst : instStor_) {
  if (inst.isDummy()) {
    dummyInsts_.push_back(&inst);
    nonPlaceInsts_.push_back(&inst);
    nonPlaceInstsArea_ += inst.getArea();
  }
  pb_insts_.push_back(&inst);
}
```

---

### 阶段 4：信息打印与利用率校验 — `printInfo()`

```
printInfo(check_density):
  ├── GPL-6  ~ 11:  实例统计（总/可移动/固定/虚拟）+ 线网数 + 引脚数
  ├── GPL-12 ~ 13:  Die BBox / Core BBox
  ├── GPL-14 ~ 15:  Region 名称与面积
  ├── GPL-16 ~ 18:  Core 面积 / 固定实例面积 / 可移动实例面积
  ├── GPL-19:       利用率:
  │     util = placeInstsArea_ / (region_area_ - nonPlaceInstsArea_) × 100
  ├── GPL-20 ~ 21:  标准单元面积 / 大单元面积
  └── 若 check_density && util >= 100.1% → GPL-301 报错
```

---

## 三、PlacerBase 内部数据结构总览

```
PlacerBase (每个 Region/Group 一个实例)
  │
  ├── 引用
  │   ├── db_         ── ODB 数据库句柄
  │   ├── log_        ── 日志器
  │   ├── pbCommon_   ── 共享的 PlacerBaseCommon (shared_ptr)
  │   └── group_      ── 关联的 ODB Group (nullptr 表示顶层)
  │
  ├── 区域几何
  │   ├── die_          ── Die 信息（从 pbCommon_ 拷贝）
  │   ├── region_bbox_  ── 区域包围盒 (Rect)
  │   ├── region_area_  ── 区域面积
  │   ├── siteSizeX_    ── 站点宽度
  │   └── siteSizeY_    ── 站点高度
  │
  ├── 实例分类
  │   ├── pb_insts_       ── 该区域所有实例 (Instance*)
  │   ├── instStor_       ── 虚拟实例存储 (Instance 对象)
  │   ├── placeInsts_     ── 可移动实例 (Instance*)
  │   ├── fixedInsts_     ── 固定实例 (Instance*)
  │   ├── dummyInsts_     ── 虚拟实例 (Instance*)
  │   └── nonPlaceInsts_  ── fixedInsts_ + dummyInsts_ (快速迭代用)
  │
  └── 面积统计
      ├── placeInstsArea_    ── 可移动实例总面积
      ├── nonPlaceInstsArea_ ── 非放置实例总面积
      ├── macroInstsArea_    ── Macro 实例面积
      └── stdInstsArea_      ── 标准单元面积
```

外部调用关系：

```
Replace::pbVec_ [vector<shared_ptr<PlacerBase>>]
  │
  ├── pbVec_[0]  ── 顶层区域 PlacerBase
  ├── pbVec_[1]  ── Region A / Group X 的 PlacerBase
  ├── pbVec_[2]  ── Region B / Group Y 的 PlacerBase
  └── ...           (每个物理 Region 下的每个 Group 各一个)
```

---

## 四、关键设计要点

### 1. 双层结构解耦全局与区域

`PlacerBaseCommon` 负责一次性的全局数据加载（实例/引脚/线网），`PlacerBase` 负责按 Region/Group 做轻量化过滤。避免为每个区域重复从 ODB 加载数据。

### 2. Dummy 实例填补不可用站点

将碎片行、Blockage、Macro Halo 等占据的不可用站点显式建模为 `Instance`（`dbInst_ == nullptr`），确保密度网格连续，使密度惩罚在这些区域正确生效。

### 3. 区域的实例归属过滤

通过 Group 匹配判断实例是否属于当前区域。顶层区域接收所有无 Group 或 `VISUAL_DEBUG` Group 的实例；子区域仅接收精确匹配 Group 的实例，实现分区布局约束。

### 4. 面积分类跟踪

将可移动实例按 `macroInstsArea_` / `stdInstsArea_` 分开统计，便于后续 `target_density` 的差异化调优。固定实例仅统计与 Core 重叠的部分。

### 5. 利用率前置校验

初始化完成后立即检查利用率，防止在无效设计上运行后续昂贵的 Nesterov 布局迭代。
