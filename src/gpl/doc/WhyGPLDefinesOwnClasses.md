# 为什么 GPL 要单独定义 class Instance, Pin, Net 而不是直接使用 ODB 中的 class？

> **源文件**: `src/gpl/src/placerBase.h` (GPL 轻量类定义), `src/gpl/src/placerBase.cpp` (初始化与转换层)
> **对比对象**: `src/odb/include/odb/db.h` (ODB 公开 API), `src/odb/src/db/dbInst.h`, `src/odb/src/db/dbNet.h` 等 (ODB 内部数据结构)
> **模块**: OpenROAD GPL (Global Placement) 全局布局器

---

## 1. 背景

OpenROAD 使用 OpenDB (ODB) 作为统一的物理设计数据库，存储芯片设计中的全部信息——实例 (instance)、网络 (net)、引脚 (pin)、布线 (wire)、RC 寄生参数、层次结构、ECO 信息等。ODB 是一个**通用数据库**，为整个工具链的所有阶段（floorplan、placement、CTS、routing、extraction、STA 等）服务。

GPL (Global Placement) 是布局模块，核心算法基于 RePlAce/ePlace 系列的解析型非线性布局方法，使用 Nesterov 加速梯度下降在 wirelength force 和 density force 之间迭代平衡。在典型的 Nesterov 优化循环中，**每次迭代都需要访问所有 instance、pin、net 的坐标信息数百万次**，循环总计运行几千次迭代。

观察 `src/gpl/src/placerBase.h` 会发现，GPL 没有直接使用 ODB 的 `dbInst`、`dbITerm`/`dbBTerm`、`dbNet` 类，而是定义了自己的 `Instance`、`Pin`、`Net` 类，同时持有 ODB 对象的原始指针作为"后引用"。本文详细分析这一设计决策的原因和权衡。

---

## 2. GPL 的轻量类定义

### 2.1 Instance (~48 字节，8 个成员变量)

```cpp
// placerBase.h:48-117
class Instance {
    odb::dbInst* inst_ = nullptr;    // 指向 ODB 对象的指针
    std::vector<Pin*> pins_;          // 属于该 instance 的引脚
    int lx_ = 0, ly_ = 0;            // 左下角坐标 (缓存)
    int ux_ = 0, uy_ = 0;            // 右上角坐标 (缓存)
    int extId_ = INT_MIN;             // 外部索引 (用于数组索引)
    bool is_macro_ = false;           // 是否为 macro cell
    bool is_locked_ = false;          // incremental placement 锁定标记
};
```

GPL Instance 只存储布局算法关心的信息：**包围盒坐标** (lx/ly/ux/uy) 和**类型标记** (macro/locked)。坐标从 ODB 初始化后缓存，之后在优化循环中直接读写而不需要再查询 ODB。

### 2.2 Pin (~48 字节，10 个成员变量 + 6 个 bit-field)

```cpp
// placerBase.h:119-192
class Pin {
    odb::dbObject* term_ = nullptr;   // 指向 ODB dbITerm 或 dbBTerm (二合一)
    Instance* inst_ = nullptr;        // 所属 Instance
    Net* net_ = nullptr;              // 所属 Net
    int cx_ = 0, cy_ = 0;            // 引脚中心绝对坐标 (缓存)
    int offsetCx_ = 0, offsetCy_ = 0; // 引脚中心相对于 instance 中心的偏移
    // 6 个 bit-field: iTermField_, bTermField_, minPinX/YField_, maxPinX/YField_
};
```

Pin 的设计有两个关键亮点：

1. **统一 ITerm 和 BTerm**：通过 `odb::dbObject* term_` 指针和 bit-field 标记，将 ODB 中两个不同的类型合并为一个 GPL 类型，避免内层循环中的类型分支。
2. **预计算偏移量**：`offsetCx_`/`offsetCy_` 存储引脚相对于 instance 中心的偏移。初始化时，`Pin::updateCoordi()` 会合并引脚的所有几何图形、应用 instance 的方向变换、计算出正确的偏移量。此后在优化循环中，引脚位置只需一次加法即可更新：

```cpp
// placerBase.cpp:522
void Pin::updateLocation(const Instance* inst) {
    cx_ = inst->cx() + offsetCx_;
    cy_ = inst->cy() + offsetCy_;
}
```

### 2.3 Net (~56 字节，6 个成员变量)

```cpp
// placerBase.h:194-227
class Net {
    odb::dbNet* net_ = nullptr;      // 指向 ODB 对象
    std::vector<Pin*> pins_;          // 该网络上的所有引脚
    int lx_ = 0, ly_ = 0;            // 包围盒左下角 (HPWL 缓存)
    int ux_ = 0, uy_ = 0;            // 包围盒右上角 (HPWL 缓存)
};
```

Net 只缓存 bounding box（用于 HPWL 快速计算），不关心 ODB net 中的 wire segments、cap nodes、RC 校准、NDR 规则等布线/提取阶段才需要的数据。

---

## 3. ODB 的完整类定义（对比）

为了理解为什么 GPL 不能直接使用 ODB，需要了解 ODB 类的真实体量。ODB 采用"内部数据 + 公开 API"两层架构，内部 `_dbXxx` 结构体持有全部数据，公开 `dbXxx` 类通过 `getImpl()` 模式访问内部数据。

### 3.1 _dbInst (内部数据，28 个成员变量)

```cpp
// dbInst.h:46-95
class _dbInst : public _dbObject {
    _dbInstFlags flags_;       // 12-bit packed: orient, status, user_flags,
                               //   physical_only, dont_touch, source, eco, level
    char* name_;
    int x_, y_, weight_;
    dbId<_dbInst>      next_entry_;       // 链表指针 (全局遍历)
    dbId<_dbInstHdr>   inst_hdr_;
    dbId<_dbBox>       bbox_;
    dbId<_dbRegion>    region_;
    dbId<_dbModule>    module_;
    dbId<_dbGroup>     group_;
    dbId<_dbInst>      region_next_, module_next_, group_next_;  // 链表指针
    dbId<_dbInst>      region_prev_, module_prev_;
    dbId<_dbHier>      hierarchy_;         // 层次化设计
    dbId<_dbChipRegion> chip_region_;
    dbId<_dbChipBump>  bump_;
    dbVector<uint32_t>  iterms_;           // 子 ITerm 的序列化向量
    dbId<_dbBox>       halo_;
    uint32_t           pin_access_idx_;
};
```

公开 `dbInst` 类暴露 **60+ 个方法**：placement、orientation、transform、状态查询（isFixed, isPlaced）、user flags、ECO flags、dont-touch、层次遍历（getChild/getParent/getChildren）、terminal 查询、connectivity 遍历、master 查询（isBlock/isCore/isPad）、weight/source、halo、region、module、group、bump、scan chain 等。

### 3.2 _dbNet (内部数据，~29 个成员变量)

```cpp
// dbNet.h:61-132
class _dbNet : public _dbObject {
    _dbNetFlags flags_;        // ~23-bit packed: sig_type, wire_type, special,
                               //   wild_connect, wire_ordered, disconnected,
                               //   spef, select, mark, wire_altered,
                               //   extracted, rc_graph, io, dont_touch, etc.
    char* name_;
    union { float gndc_calibration_factor_; float ref_cc_; };
    union { float cc_calibration_factor_; float db_cc_; float cc_match_ratio_; };
    dbId<_dbNet>     next_entry_;
    dbId<_dbITerm>   iterms_;               // ITerm 双向链表头
    dbId<_dbBTerm>   bterms_;               // BTerm 双向链表头
    dbId<_dbWire>    wire_, global_wire_;
    dbId<_dbSWire>   swires_;
    dbId<_dbCapNode> cap_nodes_;            // RC 寄生节点
    dbId<_dbRSeg>    r_segs_;               // RC 寄生段
    dbId<_dbTechNonDefaultRule> non_default_rule_;
    dbId<_dbGuide>   guides_;
    dbId<_dbNetTrack> tracks_;
    dbVector<dbId<_dbGroup>> groups_;
    int weight_, xtalk_;
    float cc_adjust_factor_;
    uint32_t cc_adjust_order_;
    int driving_iterm_;
};
```

公开 `dbNet` 类暴露 **100+ 个方法**：信号/时钟/电源类型、约 10 个 boolean 标记、RC 寄生（cap nodes, RSegs, CCSegs 的增删改查）、校准与调整、buffer 插入、net 合并、层次化连接、静态工厂方法等。

### 3.3 _dbITerm + _dbBTerm (内部数据，~11 + 20 个成员变量)

`_dbITerm` 持有 flags (mterm_idx, clocked, mark, spef, special, connected)、`net_`/`mnet_`/`inst_`、net 和 modnet 上的双向链表指针、`mtem_` 缓存、STA vertex id、access points 映射表。

`_dbBTerm` 更加复杂：额外持有 `parent_block_`/`parent_iterm_`（层次跨越）、`bpins_`（物理引脚形状）、`ground_pin_`/`supply_pin_`（敏感引脚）、`constraint_region_`、`mirrored_bterm_`、`chip_region_`/`chip_bump_` 等。

### 3.4 对比小结

| 指标 | dbInst (ODB) | GPL Instance | dbNet (ODB) | GPL Net | dbITerm+dbBTerm (ODB) | GPL Pin |
|------|-------------|-------------|------------|---------|----------------------|---------|
| **内部成员变量** | ~28 | 8 | ~29 | 6 | ~11 + ~20 | 10 |
| **公开方法 (约)** | ~60 | ~15 | ~100 | ~9 | ~25 + ~35 | ~20 |
| **每对象内存 (约)** | ~200+ 字节 | ~48 字节 | ~200+ 字节 | ~56 字节 | ~120+ 字节 | ~48 字节 |
| **链表指针** | 7 个 | 0 | 2 个 | 0 | 4 + 5 个 | 0 (裸 C++ 指针) |
| **层次支持** | 完整 | 无 | 完整 + modnet | 无 | parent/child 穿越 | 无 |
| **RC/提取** | N/A | N/A | 完整 | 无 | N/A | N/A |

ODB 每个类大约是 GPL 对应类的 **3-10 倍大小**，且带有大量布局阶段不需要的链路关系（双向链表、层次连接、RC 数据、校准参数等）。

---

## 4. 为什么 GPL 必须定义自己的轻量类

### 4.1 性能关键路径：Nesterov 优化循环

GPL 的核心算法（ePlace-MS）执行 Nesterov 加速梯度下降，典型流程为：

```
for iter in 1..maxNesterovIter (通常几千次):
    updateWireLengthForceWA()      # 遍历所有 net、pin，计算基于 WA 模型的线长梯度
    updateDensityFieldBin()        # 遍历所有 gcell，更新密度场 (FFT/Poisson)
    updateGradients()              # 遍历所有 gcell，累加线长梯度和密度梯度
    nesterovUpdateCoordinates()    # 更新所有 gcell 的坐标
    checkConvergence()             # 检查收敛条件
```

每次 `updateWireLengthForceWA()` 需要访问所有引脚的坐标（`cx_`/`cy_`）来计算 WA (Weighted Average) 线长模型中的加权指数和。对于一个百万实例规模的设计，这意味着**每步迭代有数百万到数亿次坐标读取**。

如果直接使用 ODB：
- 每次获取坐标需要 `inst->getBBox()->xMin()` 这样的链式调用，涉及指针跳转、dbBox 对象查找
- ITerm 的引脚位置需要合并几何图形并做方向变换，无法缓存
- ODB 对象在内存中不连续（通过链表和哈希表组织），cache miss 率高

GPL 的做法：
- 坐标是类内第一个数据成员，直接内联访问 `inst->lx()`
- 引脚位置预计算为 `inst->cx() + offsetCx_`，一次整数加法
- 所有对象存储在 `std::vector` 中，内存连续，cache 友好

**性能差异：在 inner loop 中，GPL 访问 Instance 坐标约 1-2 个 CPU 周期（L1 cache hit），而 ODB 路径可能需要 50-200 个周期（多次指针跳转 + 可能的 cache miss）。对于数千次迭代 × 数百万 instances，这个差距是巨大的。**

### 4.2 Cache 局部性与内存布局

GPL 将所有 Instance/Pin/Net 存储在连续的 `std::vector` 中：

```cpp
// placerBase.h:329-335
std::vector<Instance> instStor_;    // 连续存储
std::vector<Pin> pinStor_;          // 连续存储
std::vector<Net> netStor_;          // 连续存储

std::vector<Instance*> insts_;      // 指针视图
std::vector<Pin*> pins_;
std::vector<Net*> nets_;
```

这意味着迭代所有实例时，CPU 预取器可以高效地将后续数据提前加载到 L1/L2 cache 中。相比之下，ODB 对象通过链表连接，每个对象可能分配在内存中不连续的位置，迭代效率低一个数量级。

特别值得注意的是 GPL Instance 只有 **48 字节**——恰好适合大多数 CPU 的一个 cache line（64 字节）。在遍历 instances 数组时，几乎每个 instance 都是一次 L1 cache hit。

### 4.3 预计算：一次性计算、循环内零开销使用

这是 GPL 设计中最精巧的部分之一。`Pin::updateCoordi()` 在初始化时执行复杂的几何计算：

```cpp
// placerBase.cpp:456-500
void Pin::updateCoordi(odb::dbITerm* iTerm) {
    // 1. 合并 MTerm 所有 MPin 的所有几何图形
    // 2. 应用 instance 的旋转/镜像变换
    // 3. 计算 pin bbox 中心相对于 instance 中心的偏移量
    offsetCx_ = pin_bbox.xCenter() - instCenterX;
    offsetCy_ = pin_bbox.yCenter() - instCenterY;
    // 4. 计算绝对坐标
    cx_ = lx + instCenterX + offsetCx_;
    cy_ = ly + instCenterY + offsetCy_;
}
```

这涉及多次 `dbBox` 遍历、`dbTransform` 矩阵运算、`merge` 操作——**每一个都是昂贵的操作**。但只执行一次。在后续的优化循环中，更新引脚位置仅仅是：

```cpp
cx_ = inst->cx() + offsetCx_;  // 一条整数加法指令
```

如果直接使用 ODB，每次需要引脚坐标时都必须重新执行上述复杂计算，或者 ODB 本身必须维护缓存（但它不这样做，因为 ODB 是"数据源"而非"计算引擎"）。

### 4.4 统一异构对象：Dummy Instance 与 Filler Cell

GPL 用同一个 `Instance` 类统一表示三种完全不同的概念：

| 类型 | `inst_` 值 | 含义 |
|------|-----------|------|
| 真实可移动实例 | `!= nullptr`, `!isFixed()` | 需要布局的标准单元/macro |
| 真实固定实例 | `!= nullptr`, `isFixed()` | 不可移动的 macro、tap cell 等 |
| Dummy instance | `== nullptr` | 不可用 site 的占位填充 |

Dummy instance 是 ODB 不存在的概念。GPL 用它们来占据不能布局的区域（碎片化的 row 段、placement blockage、macro halo 区域、region 预留区域等），确保 bin grid 密度计算正确反映可用空间。所有三种类型的 Instance 被统一放入 bin grid，密度场计算不需要区分它们。

同样，`Pin` 统一处理 ITerm 和 BTerm：

```cpp
odb::dbObject* term_ = nullptr;   // 可以是 dbITerm* 或 dbBTerm*
unsigned char iTermField_ : 1;    // 标记是哪种类型
unsigned char bTermField_ : 1;
```

这避免了在 wirelength gradient 计算中做类型判断分支。

### 4.5 Placement 特有的操作

GPL 需要在 Instance 上执行一些 ODB 没有原生的操作：

- **`extendSizeByScale(scale)`**：在 routability-driven 模式下，根据引脚密度对 instance 进行膨胀/收缩，同时自动更新所有关联引脚的坐标。ODB 没有"缩放 instance 面积"的语义。
- **`snapOutward(origin, step_x, step_y)`**：固定 instance 向外吸附到 site 边界，确保部分覆盖的 site 被标记为完全占用。ODB 的 `setLocation()` 不做 site 对齐。
- **`padLeft`/`padRight`**：GPL 允许用户指定在 instance 左右各扩展 N 个 site，这在 congestion 预估中被使用。ODB 不清楚 site grid 的存在。
- **`lock()`/`unlock()`**：incremental placement 时临时固定某些 instance，这是 GPL 层面的概念，不应写入 ODB。

### 4.6 Extract-Transform-Load (ETL) 架构模式

整体设计遵循数据库系统中的 ETL 模式：

```
                    init()                            优化循环                      dbSetLocation()
ODB (数据源)  ──────────────>  GPL Instance/Pin/Net  ──────────>  优化后的坐标    ──────────────>  ODB (写回)
             提取 + 变换          (优化专用结构)        (快速计算)                    写回
```

- **Extract**：`PlacerBaseCommon::init()` 遍历 ODB 的所有 dbInst/dbNet/dbITerm/dbBTerm，提取布局相关的信息（位置、尺寸、连接关系），同时过滤掉不相关的数据（电源网络、RC 寄生、层次结构等）。
- **Transform**：预计算引脚偏移量（含旋转变换）、处理 site 对齐、创建 dummy instance 填充不可用区域、根据引脚密度进行 instance 缩放。
- **Load**：优化完成后，`Instance::dbSetLocation()` 将计算出的坐标写回 ODB。

这种分离使得：
- ODB 保持为 single source of truth，不会被优化算法的中间状态污染
- 如果布局失败（发散、overflow 超标），可以直接丢弃 GPL 数据重新开始，不影响 ODB
- 多个 PlacerBase (每个对应一个 region/group) 可以共享一个 PlacerBaseCommon

---

## 5. 转换/适配层

### 5.1 正向转换 (ODB → GPL)：`PlacerBaseCommon::init()`

位于 `placerBase.cpp:752-998`。执行顺序：

```
日志 → 站点与 Die/Core → 填充实例 → 引脚密度统计 → 按密度扩展
    → 构建实例索引 → 填充网络/引脚 → 构建引脚索引
    → 实例挂引脚 → 网络挂引脚
```

关键过滤逻辑：
- **Instance**：只保留 `CORE` 和 `BLOCK` 类型的 master（过滤 PAD、ENDCAP 等）
- **Net**：只保留 `SIGNAL` 和 `CLOCK` 类型（过滤 VDD、VSS 等电源网络）
- **Pin**：只保留信号网络上的引脚，电源引脚被丢弃

### 5.2 反向查找 (ODB → GPL)：`dbToPb()` 系列

```cpp
// placerBase.h:300-303
Instance* PlacerBaseCommon::dbToPb(odb::dbInst* inst) const;
Pin*      PlacerBaseCommon::dbToPb(odb::dbITerm* term) const;
Pin*      PlacerBaseCommon::dbToPb(odb::dbBTerm* term) const;
Net*      PlacerBaseCommon::dbToPb(odb::dbNet* net) const;
```

使用 `boost::unordered_flat_map` 实现 O(1) 查找。这些方法仅在初始化和写回阶段使用，不在优化循环中调用。

### 5.3 写回 (GPL → ODB)：`Instance::dbSetLocation()`

```cpp
// placerBase.cpp:184-188
void Instance::dbSetLocation(int x, int y) {
    setLocation(x, y);         // 更新 GPL 内部坐标
    dbSetLocation();           // inst_->setLocation(lx_, ly_)
}
```

写回是显式的、手动的——不存在自动同步。这避免了优化循环中的不必要 ODB 访问。

---

## 6. 权衡分析

### 6.1 获得的好处

| 好处 | 说明 |
|------|------|
| **大幅度性能提升** | 内层循环中坐标访问从 50-200 CPU cycles 降到 1-2 cycles |
| **Cache 友好** | 连续内存布局，CPU 预取高效 |
| **预计算复用** | 复杂的几何变换只做一次，循环内零开销 |
| **统一异构对象** | dummy instance、filler cell、real instance 用同一接口处理 |
| **算法无关存储** | 为布局算法定制数据布局，不受 ODB 数据模型约束 |
| **隔离性** | ODB 不会被优化中间状态污染；发散/失败可安全回滚 |

### 6.2 付出的代价

| 代价 | 说明 |
|------|------|
| **额外内存** | 对于 N 个 instance、M 个 pin、K 个 net，GPL 额外分配 ~48N + 48M + 56K 字节（但相比 ODB 自身的 ~200N + 120M + 200K 字节，比例不大） |
| **一致性维护** | GPL 的 `lx_/ly_/ux_/uy_` 不会自动与 ODB 同步，需要通过 `dbSetLocation()` 显式写回 |
| **重建成本** | 当 timing-driven 迭代中 resizer 改变 cell 大小时，GPL 数据结构需要通过 callback 系统增量更新（`NesterovBaseCommon` 的 callback 机制） |
| **额外间接层** | `dbToPb()` 查找需要 hash map 查询，但仅在初始化/写回时使用，非性能关键 |
| **代码复杂度** | 维护两套并行数据结构增加了理解和维护负担 |

### 6.3 为什么这个权衡是值得的

在现代 VLSI 设计中，百万实例级别的 global placement 是**计算密集型**任务。Nesterov 优化循环的运行时间主要由以下因素决定：

- **坐标访问频率**：每步迭代 O(N_pins) 次坐标读取
- **Cache miss 成本**：一次 L3 cache miss (~40ns) 相当于 ~120 条 CPU 指令

如果直接使用 ODB 类，每个坐标访问都涉及多级指针跳转（`inst->getBBox()->xMin()`），每次都可能触发 cache miss。对于 1000 次迭代 × 500 万个引脚，这意味着数十亿次不必要的 cache miss，可能增加**数倍到数十倍**的运行时间。

GPL 用约 **~48×(N+M+K) 字节** 的额外内存，换取了内层循环**近乎 100% L1 cache hit rate** 的访问模式。在典型的设计规模下，这个额外内存开销约为几十到几百 MB，相比 ODB 自身的内存占用（通常数 GB）是可以接受的。

---

## 7. 与其他模块的对比

GPL 的做法在 OpenROAD 中并非孤例。其他模块也采用了类似的模式：

- **Resizer (rsz)**：虽不定义完整的 Instance 类，但维护自己的 instance 缓存和时序图，不直接依赖 ODB 遍历
- **TritonRoute (drt)**：定义自己内部的网络和引脚表示，将 ODB 数据转换为路由专用的图结构
- **OpenSTA**：维护完全独立的时序图 (timing graph)，从 ODB 中提取连接关系但使用自己的数据结构

这是 EDA 工具设计中的常见模式：**将通用数据库转换为专用数据结构的"阻抗匹配"层**。

---

## 8. 总结

GPL 单独定义 `Instance`、`Pin`、`Net` 类的根本原因是 **ODB 作为通用物理设计数据库，其数据模型和访问模式与解析型布局算法的需求不匹配**。

具体来说：

1. **ODB 太"重"**：每个对象携带大量布局不需要的数据（层次关系、RC 寄生、链表指针、校准参数等），对象大小是 GPL 轻量类的 3-10 倍。
2. **ODB 不连续**：对象通过链表和哈希表组织，遍历时 cache miss 率高；GPL 使用连续 `std::vector`，CPU 预取效率高。
3. **ODB 不做预计算**：引脚坐标需要每次都从几何图形合并并做方向变换；GPL 在初始化时预计算一次，循环内只需整数加法。
4. **ODB 缺少布局概念**：dummy instance（不可用 site 填充）、instance 缩放、site 对齐等布局特有操作在 ODB 中没有对应语义。
5. **隔离性需求**：optimization 的中间状态不应污染 ODB，失败时可以直接丢弃 GPL 数据。

这种设计遵循 **Extract-Transform-Load 模式**：初始化时从 ODB 提取并变换数据，优化循环在 GPL 自有结构上高效运行，最终将结果写回 ODB。这几十到几百 MB 的额外内存，换来了**数倍到数十倍的计算加速**，对于百万实例规模的 VLSI global placement 是完全值得的。
