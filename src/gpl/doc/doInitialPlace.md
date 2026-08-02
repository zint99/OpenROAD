# Replace::doInitialPlace 技术文档

## 1. 概述

`Replace::doInitialPlace` 是 OpenROAD GPL (Global Placement) 模块中**初始布局**的入口函数。它负责在合法化（legalization）和详细布局（detailed placement）之前，为所有可布放的标准单元实例计算一个近似最优的初始位置。

该函数采用**基于 B2B (Bound-to-Bound) 网络模型的共轭梯度求解器 (BiCGSTAB)**，通过迭代求解稀疏线性方程组来最小化线长。

- **源文件**: `src/gpl/src/replace.cpp:215-250`
- **声明**: `src/gpl/include/gpl/Replace.h:133`
- **核心求解器**: `src/gpl/src/initialPlace.cpp:50-124`

---

## 2. 函数签名

```cpp
void Replace::doInitialPlace(const int threads, const PlaceOptions& options);
```

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `threads` | `const int` | OpenMP 线程数，传递给 Eigen BiCGSTAB 求解器用于并行计算 |
| `options` | `const PlaceOptions&` | 布局选项结构体，包含所有可配置的布局参数（迭代次数、权重、密度等）。有默认值 `{}` |

### PlaceOptions 关键参数（用于初始布局的部分）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `initialPlaceMaxIter` | `int` | 20 | 外循环最大迭代次数 |
| `initialPlaceMinDiffLength` | `int` | 1500 | B2B 模型中 pin 间距的最小值（DBU），防止除零和权重爆炸 |
| `initialPlaceMaxSolverIter` | `int` | 100 | BiCGSTAB 求解器每次内循环的最大迭代次数 |
| `initialPlaceMaxFanout` | `int` | 200 | 参与 B2B 建模的最大 fanout，超过此值的 net 被跳过 |
| `initialPlaceNetWeightScale` | `float` | 800 | B2B 模型中的线网权重缩放因子 |
| `forceCenterInitialPlace` | `bool` | false | 强制所有实例初始位置为芯片中心（忽略 DB 中已有的位置） |

---

## 3. 执行流程详解

### 3.1 整体调用链

```
Replace::doInitialPlace(threads, options)
  │
  ├── [1] checkHasCoreRows()
  │
  ├── [2] 首次调用：构建 PlacerBaseCommon + PlacerBase 对象
  │
  ├── [3] 提取 InitialPlaceVars
  │
  └── [4] InitialPlace::doBicgstabPlace(threads)
        │
        ├── [4.1] placeInstsInitialPositions()     初始化实例位置
        ├── [4.2] setPlaceInstExtId()               分配线性索引
        │
        └── [4.3] for iter = 1 .. maxIter:
              ├── updatePinInfo()                   更新 B2B pin 标记
              ├── createSparseMatrix()              构建稀疏力矩阵
              ├── cpuSparseSolve()                  BiCGSTAB 求解
              └── updateCoordi()                    回写坐标到 DB
```

---

### 3.2 第 1 步：`checkHasCoreRows()` (行 217)

```cpp
checkHasCoreRows();
```

验证芯片数据库中已定义 core rows。如果未定义，直接报错退出（错误码 130），因为布局必须在有行（rows）的区域内进行。

---

### 3.3 第 2 步：首次调用构建 PlacerBase (行 218-242)

```cpp
if (pbc_ == nullptr) {
    pbc_ = std::make_shared<PlacerBaseCommon>(db_, options, log_);
    // ... 构建 pbVec_
}
```

这是一个**懒初始化（lazy initialization）**守卫：`pbc_` 为 `nullptr` 表示第一次调用 `doInitialPlace`，此时需要构建整个数据模型。后续重复调用（例如在 Nesterov 布局的多次迭代之间）会跳过此步。

#### 构建过程

1. **创建 `PlacerBaseCommon`**：从 `odb::dbDatabase` 中提取芯片的 die 边界、core 区域、所有实例、所有 net 及其 pin 连接关系，构建为布局器内部的数据结构。
2. **创建顶层 `PlacerBase`**：为顶层（top-level）区域创建 `PlacerBase`，`true` 参数表示放置实例到 core 区域中心。
3. **为每个 region group 创建 `PlacerBase`**：遍历 DB 中所有 region 的所有 group，为每个 group 创建独立的 `PlacerBase`（region-aware 布局）。
4. **清理空的顶层 `PlacerBase`**：如果顶层没有任何可放置实例（全部被分配到 region group 中），则移除顶层 `PlacerBase` 并发出警告（GPL-123）。
5. **统计总可放置实例数** (`total_placeable_insts_`)：遍历所有 `PlacerBase` 累加器实例数。

---

### 3.4 第 3 步：提取 `InitialPlaceVars` (行 244)

```cpp
const InitialPlaceVars ipVars(options, gui_debug_initial_);
```

`InitialPlaceVars` 是 `InitialPlace` 求解器专属的参数结构体（定义于 `src/gpl/src/initialPlace.h:24-35`），从 `PlaceOptions` 中提取初始布局需要的关键参数：

```cpp
struct InitialPlaceVars {
    const int maxIter;           // = options.initialPlaceMaxIter (默认 20)
    const int minDiffLength;     // = options.initialPlaceMinDiffLength (默认 1500)
    const int maxSolverIter;     // = options.initialPlaceMaxSolverIter (默认 100)
    const int maxFanout;         // = options.initialPlaceMaxFanout (默认 200)
    const float netWeightScale;  // = options.initialPlaceNetWeightScale (默认 800)
    const bool debug;            // = gui_debug_initial_
    const bool forceCenter;      // = options.forceCenterInitialPlace (默认 false)
};
```

所有成员都是 `const`，确保求解过程中参数不可变更。

---

### 3.5 第 4 步：核心求解 `InitialPlace::doBicgstabPlace()` (行 246-249)

```cpp
std::unique_ptr<InitialPlace> ip(
    new InitialPlace(ipVars, pbc_, pbVec_, graphics_->MakeNew(log_), log_));
ip_ = std::move(ip);
ip_->doBicgstabPlace(threads);
```

创建 `InitialPlace` 求解器对象，移动到成员变量 `ip_`（确保生命周期），然后调用核心方法。

---

## 4. BiCGSTAB 初始布局算法详解

`InitialPlace::doBicgstabPlace()` 位于 `src/gpl/src/initialPlace.cpp:50-124`，是整个初始布局的核心。

### 4.1 初始化实例位置：`placeInstsInitialPositions()`

为每个可放置实例设定初始坐标。按优先级选择位置来源：

| 优先级 | 条件 | 位置来源 |
|--------|------|----------|
| 1 | 实例锁定的 (`isLocked()`) | 跳过，不修改 |
| 2 | 实例属于 region group | region 的几何中心 |
| 3 | `!forceCenter` 且 DB 中已放置 (`isPlaced()`) | DB 中记录的原始坐标 |
| 4 | 其他所有情况 | core 区域的几何中心 |

最后输出统计日志（GPL-51），显示每种位置来源的实例数量。

### 4.2 分配扩展 ID：`setPlaceInstExtId()`

为每个可放置实例分配从 0 开始的线性索引 (`ExtId`)。该索引用于将实例映射到稀疏矩阵和向量的行/列位置。不可放置的实例（如 fixed macro）的 `ExtId` 设为 `INT_MAX`。

### 4.3 主迭代循环

```cpp
for (size_t iter = 1; iter <= ipVars_.maxIter; iter++) {
    updatePinInfo();
    createSparseMatrix();
    error = cpuSparseSolve(...);
    updateCoordi();
    // 收敛检查
    if (error_max <= 1e-5 && iter >= 5) break;
}
```

#### 4.3.1 `updatePinInfo()` — 更新 B2B Pin 标记

对每个 net，在 X 和 Y 方向分别找出具有最小/最大坐标的 pin，并标记为 `minPinX`/`maxPinX`/`minPinY`/`maxPinY`。

**B2B (Bound-to-Bound) 模型**：对于 HPWL (Half-Perimeter Wire Length)，线长由 bounding box 的对角两点决定：
$$
\text{HPWL} = (x_{\max} - x_{\min}) + (y_{\max} - y_{\min})
$$

因此，只有位于 bounding box 边界上的 pin（min/max pin）对线长有影响，内部 pin 不参与力的计算。

#### 4.3.2 `createSparseMatrix()` — 构建稀疏力矩阵

这是算法最核心的部分。对于每个 net，考虑其 B2B pin 对，构建两个独立的稀疏线性系统（X 和 Y 方向解耦）。

**数学模型**：

对于 X 方向，目标是最小化加权 HPWL。对每个 B2B pin 对 $(p_i, p_j)$，其 X 方向的代价贡献为：

$$
\text{cost}_{ij}^x = w_{ij} \cdot |x_i + \Delta x_i - (x_j + \Delta x_j)|
$$

其中：
- $w_{ij} = \dfrac{\text{netWeightScale}}{\text{fanout} - 1} \cdot \dfrac{1}{\max(d_{ij}, \text{minDiffLength})}$
- $\Delta x_i, \Delta x_j$ 是实例中心到 pin 的偏移量
- $d_{ij} = |p_i.cx() - p_j.cx()|$ 是 pin 之间的当前距离

**线性系统**：
$$
A \cdot \vec{x} = \vec{b}
$$

对 X 方向，矩阵 $A$ 的元素由以下规则生成（以两个可移动实例为例）：

```
A(inst1, inst1) +=  weightX
A(inst2, inst2) +=  weightX
A(inst1, inst2) += -weightX
A(inst2, inst1) += -weightX
```

RHS 向量 $\vec{b}$ 包含固定实例和 IO port 施加的"拉力"：

```
b(inst1) += -weightX * ((pin1.cx - inst1.cx) - (pin2.cx - inst2.cx))
b(inst2) += -weightX * ((pin2.cx - inst2.cx) - (pin1.cx - inst1.cx))
```

**特殊情况处理**：

| 情况 | 处理方式 |
|------|----------|
| 一个可移动 + 一个固定实例 | 仅给可移动实例添加对角项，固定实例坐标计入 RHS |
| 一个可移动 + 一个 IO port | 同上，IO port 的绝对坐标计入 RHS |
| 两个固定实例 | 完全跳过，无贡献 |
| 同一实例的两个 pin | 跳过，不影响实例坐标 |

**性能优化**：
- 使用 Eigen 的 triplet 列表方式构建稀疏矩阵（`setFromTriplets`），内存高效
- 预先 reserve 100 万元素的向量容量，避免动态扩容
- fanout ≥ `maxFanout` (默认 200) 的 net 直接跳过
- 单 pin net 跳过

#### 4.3.3 `cpuSparseSolve()` — BiCGSTAB 求解器

位于 `src/gpl/src/solver.cpp:12-49`。

```cpp
omp_set_num_threads(threads);
BiCGSTAB<SMatrix, IdentityPreconditioner> solver;
solver.setMaxIterations(maxSolverIter);

// 求解 X 方向
solver.compute(placeInstForceMatrixX);
instLocVecX = solver.solveWithGuess(fixedInstForceVecX, instLocVecX);

// 求解 Y 方向
solver.compute(placeInstForceMatrixY);
instLocVecY = solver.solveWithGuess(fixedInstForceVecY, instLocVecY);
```

关键设计决策：

- **BiCGSTAB (Biconjugate Gradient Stabilized)**：比标准 CG 更适合非对称矩阵，收敛更平滑
- **Identity Preconditioner**：不使用预条件器，因为 B2B 模型的矩阵条件数已经足够好
- **solveWithGuess**：使用上一次迭代的解作为初值（warm start），显著加速收敛
- **X/Y 解耦**：X 和 Y 方向完全独立求解，两个方向可并行化（但当前实现是串行的）
- **线程数控制**：通过 `omp_set_num_threads(threads)` 设置 OpenMP 线程数，Eigen 内部利用多线程加速 SpMV 操作

**残差计算**：求解器返回相对残差 $\varepsilon$，如果求解器状态为 `NoConvergence` 或 `Success`，则使用 `solver.error()` 的值；否则（数值错误）返回 `NaN`。

#### 4.3.4 收敛检查

```cpp
if (std::isnan(error.x) || std::isnan(error.y)) {
    // 数值错误，提前退出
    break;
}

float error_max = std::max(error.x, error.y);
if (error_max <= 1e-5 && iter >= 5) {
    break;  // 收敛
}
```

收敛条件：**最大残差 ≤ 1e-5 且迭代次数 ≥ 5**。最小 5 次迭代保证了即使初始误差就很小，也不会过早退出。

#### 4.3.5 `updateCoordi()` — 回写坐标

将求解器得到的坐标向量写回数据库实例：

1. 从 `instLocVecX_` 和 `instLocVecY_` 读取新坐标
2. **钳制到 core 区域**：坐标不允许超出 `[coreLx, coreUx] × [coreLy, coreUy]`
3. **Region 约束**：如果实例属于 region group，进一步钳制到 region 的边界框内
4. 调用 `dbSetCenterLocation()` 和 `dbSetPlaced()` 更新 DB 状态

---

## 5. 算法特性总结

### 5.1 设计优势

| 特性 | 说明 |
|------|------|
| **B2B 模型** | 只考虑 bounding box 边界上的 pin，大幅减少矩阵非零元素 |
| **X/Y 解耦** | 两个方向独立求解，矩阵规模减半，求解更快 |
| **Warm start** | 每次迭代用前次解作为初值，迭代次数递减 |
| **Sparse matrix** | Eigen 稀疏矩阵，内存和计算效率高 |
| **Inverse distance weighting** | 权重与 pin 间距成反比，防止长线过度拉动实例 |
| **MinDiffLength 稳定化** | 防止过近 pin 对产生极端权重 |

### 5.2 局限性

- 模型仅优化线长（wirelength），不直接优化密度。密度优化由后续的 Nesterov 布局负责
- B2B 模型是 HPWL 的近似，不是连续可微的精确线长
- 跳过超高 fanout net（如时钟网络），这些需要专门的时钟树综合
- Identity preconditioner 对于病态问题可能收敛慢

### 5.3 与后续阶段的关系

```
doInitialPlace (共轭梯度 B2B 模型，输出初始坐标)
    │
    ▼
doNesterovPlace (Nesterov 梯度下降，优化密度 + 线长)
    │
    ▼
详细布局 & 合法化
```

初始布局阶段不关注密度，只关注连接的"拉力"平衡，输出一个在连通性意义上合理的初始解。

---

## 6. 关键数据结构

| 数据结构 | 文件 | 用途 |
|----------|------|------|
| `Replace` | `src/gpl/include/gpl/Replace.h:111` | GPL 顶层接口类 |
| `PlaceOptions` | `src/gpl/include/gpl/Replace.h:48` | 所有布局参数的配置结构体 |
| `InitialPlaceVars` | `src/gpl/src/initialPlace.h:24` | 初始布局求解器的参数子集 |
| `InitialPlace` | `src/gpl/src/initialPlace.h:39` | BiCGSTAB 初始布局求解器类 |
| `PlacerBaseCommon` | `src/gpl/src/PlacerBaseCommon.h` | 从 odb 提取的共享布局数据（instances, nets, pins, die 信息） |
| `PlacerBase` | `src/gpl/src/PlacerBase.h` | 单个区域（顶层或 region group）的布局数据 |
| `SMatrix` | `src/gpl/src/initialPlace.h:37` | `Eigen::SparseMatrix<float, RowMajor>` 的别名 |
| `ResidualError` | `src/gpl/src/solver.h:20` | `{float x, y}` 结构体，保存两个方向的残差 |

---

## 7. 相关文件索引

| 文件 | 内容 |
|------|------|
| `src/gpl/src/replace.cpp:215-250` | `doInitialPlace` 函数体 |
| `src/gpl/src/replace.cpp:113-121` | `checkHasCoreRows` |
| `src/gpl/src/initialPlace.cpp:25-35` | `InitialPlaceVars` 构造函数 |
| `src/gpl/src/initialPlace.cpp:50-124` | `doBicgstabPlace` 主循环 |
| `src/gpl/src/initialPlace.cpp:127-180` | `placeInstsInitialPositions` |
| `src/gpl/src/initialPlace.cpp:194-249` | `updatePinInfo` (B2B 标记) |
| `src/gpl/src/initialPlace.cpp:253-467` | `createSparseMatrix` (力矩阵构建) |
| `src/gpl/src/initialPlace.cpp:469-511` | `updateCoordi` (坐标回写) |
| `src/gpl/src/solver.cpp:12-49` | `cpuSparseSolve` (BiCGSTAB) |
| `src/gpl/src/initialPlace.h` | `InitialPlaceVars`, `InitialPlace` 类声明 |
| `src/gpl/src/solver.h` | `ResidualError`, `cpuSparseSolve` 声明 |
| `src/gpl/include/gpl/Replace.h:48-109` | `PlaceOptions` 结构体 |
