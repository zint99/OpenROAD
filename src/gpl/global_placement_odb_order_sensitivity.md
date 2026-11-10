# global_placement 对 ODB 迭代顺序的敏感性研究

研究对象：`initial_place`、`-timing_driven`、`-routability_driven`。

汇报版：[HTML presentation](global_placement_odb_order_sensitivity.html)。

源码基线：OpenROAD `016cc5e3e211521470a4ab2aac1fb21ac8dfd884`；OpenSTA 子模块 `e983e15b2cb346badf8f23725fd3914d25312c44`。分析以当前 CPU 实现、普通单 block 设计为主；不把结论扩展为 GPU、3DIC、层次化网络所有分支的完整审计。

## 1. 结论

**三个功能的完整布局流程均不能保证：在设计语义不变、只改变 ODB 接口遍历顺序时，QoR 保持不变。** 但三者的直接原因不同，不能笼统归因于“遍历了 ODB”或者“没有排序”。

| 功能 / 路径 | 功能自身的顺序敏感性 | 对 QoR 的含义 | 证据强度 |
| --- | --- | --- | --- |
| `initial_place` | **有，包含离散建模差异**。net 内并列极值 pin 由首次遇到者胜出；net 顺序影响浮点装配；instance 顺序影响变量编号和数值求解 | 可改变 B2B 方程、初始坐标和后续优化起点 | 源码闭环；精确算术反例验证一轮求解及模型 HPWL 不同 |
| `timing_driven` 的 slack→weight 内核 | **给定相同的逐 net slack、有效 net 集合及参数时，对遍历顺序独立**；已有 slack＋ID 排序，且按阈值计算所有 net 的权重 | 不能将 net 遍历次序直接解释为“不同 net 被赋予不同关键性权重” | 源码分析；24 种输入排列验证 |
| 完整 `timing_driven` | **有数值及间接敏感路径**。上游 STA 的负载电容按 ODB pin 顺序累加；公共 initial/Nesterov 流程还会改变输入坐标、梯度和触发时机 | slack、修复选择、权重以及最终 WNS/TNS/HPWL 可能变化；并非每次置换都会变化 | 源码闭环；浮点累加机制验证，未测真实设计上的最终 WNS/TNS 差值 |
| 默认 `routability_driven` / RUDY | **有数值敏感性**。tile demand 按 `getNets()` 顺序以 float 累加 | 可改变拥塞值；接近阈值时可改变 inflation、停止/回退与后续布局轨迹 | 源码闭环；浮点累加机制验证 |
| RUDY 可选 net reweight | **有条件的离散敏感性**。需求同分时没有第二排序键；在 `worst == cutoff` 分支尤其明显 | 被实际提高权重的 net 集合可能不同 | 源码；同分选择反例。**此功能默认关闭** |
| `routability_driven -routability_use_grt` / FastRoute | net 总表已有名称排序；**pin 遍历仍有离散敏感点**，重合 pin 去重可改变 driver root | 改变路由器输入；根敏感的 Steiner 构造等配置下可影响拥塞及后续布局 | 源码闭环；root 位置反例。未测完整路由 QoR |

这里的“敏感”表示**存在传播至 QoR 的机制，没有对任意合法置换的结果不变保证**，不表示任何设计、任何接口的一次置换都会使最终指标变化，也不表示变化一定显著。单次重复运行稳定，同样不能证明对迭代顺序独立。

按接口汇总主要传播路径如下；表中“数值”指潜在浮点舍入差异，不代表已经测得实际设计 QoR 差值：

| 只置换的接口 | initial_place | timing_driven | routability_driven |
| --- | --- | --- | --- |
| `block->getInsts()` | 变量编号置换，有限精度求解可能不同 | 公共布局初始化按实例分配随机扰动；修复 driver 顺序另有名称排序 | 同样继承公共布局初始化的扰动分配差异 |
| `block->getNets()` | float 矩阵/RHS 装配顺序 | 固定 slack 的权重分配不敏感；完整流程仍继承公共布局输入变化 | RUDY tile 累加有数值敏感性；可选 reweight 有同分选择；GRT net 总表有名称排序 |
| `net->getITerms()` / `getBTerms()` | 并列极值代表可能改变，属于离散建模差异 | STA pin 电容累加、公共 WA 梯度存在数值路径 | 固定几何的 RUDY bbox 不敏感；FastRoute pin 去重/root 有离散路径；公共 WA 梯度有数值路径 |
| `inst->getITerms()` | 当前 initial_place 的主要 pin 对遍历来自 net，不能等同于 net pin 置换 | 公共每 cell WA 梯度的累加次序可能变化 | 同样影响公共每 cell WA 梯度 |

## 2. 研究口径与 ODB 顺序语义

### 2.1 控制变量

需要区分两种实验：

1. **只改变返回顺序**：保留对象名称、ID、连接、master、尺寸、pin 几何、初始坐标、约束、参数、线程数与 seed，仅置换某个接口返回的对象序列。这是本研究的核心问题。
2. **改变创建/读入顺序后重建 ODB**：可能同时改变 ID、内部图编号和连接链表顺序，属于更强扰动。它能暴露工程上的顺序依赖，但不能把差异全部归于一个迭代器。

QoR 应按对象名称匹配比较，包含布局结束时的 HPWL、overflow、拥塞 RC，以及在相同寄生参数评估下的 WNS/TNS；不能以 DEF 文本行顺序或二进制 ODB 是否相同代替 QoR。

### 2.2 当前 ODB 并非每次随机返回

[`dbSet` 的说明](../odb/include/odb/dbSet.h#L52)区分 sequential 与 non-sequential iterator；后者没有排序性质保证。

| 接口 | 当前普通 block 实现 | 对本研究的意义 |
| --- | --- | --- |
| `dbBlock::getInsts()` | [返回 instance table](../odb/src/db/dbBlock.cpp#L1851)，[table 按 ID 递增扫描](../odb/src/db/dbTable.inc#L408) | 同一数据库的顺序通常稳定，但不是按实例语义名称规范化 |
| `dbBlock::getNets()` | [返回 net table](../odb/src/db/dbBlock.cpp#L2069) | 与 instance table 同理；不能假定 `.reverse()` 会反转它 |
| `dbNet::getITerms()` | [`dbNetITermItr`](../odb/src/db/dbITermItr.cpp#L23) 沿连接链表遍历，`sequential() == 0`，支持 `reverse()` | 可在对象/连接集合不变时真正改变 net 内 pin 顺序 |
| `dbNet::getBTerms()` | [`dbNetBTermItr`](../odb/src/db/dbBTermItr.cpp#L24) 沿链表遍历，支持 `reverse()` | 同样可能改变 net 内 pin 顺序 |
| `dbInst::getITerms()` | [`dbInstITermItr`](../odb/src/db/dbITermItr.cpp#L123) 按 instance 的 iterm 数组 / mterm index 遍历，不支持反转 | 与 `dbNet::getITerms()` 是不同迭代器，不能混用结论 |

[`dbTable::reversible()` 返回 false，`reverse()` 是空操作](../odb/src/db/dbTable.inc#L390)。所以“对 block 的 getInsts/getNets 调用 reverse 后结果相同”不是有效的反证。

## 3. 三者共享的顺序传播入口

### 3.1 ODB 顺序直接进入 GPL 容器

[`PlacerBaseCommon` 初始化](src/placerBase.cpp#L809)没有统一按名称排序：

```text
block->getInsts() → instStor_ → insts_ / placeInsts_
block->getNets()  → netStor_  → nets_
net->getITerms(), net->getBTerms() → Net::pins_
inst->getITerms() → Instance::pins_
```

具体见 [instance 收集](src/placerBase.cpp#L809)、[net/pin 收集](src/placerBase.cpp#L934)、[填充 instance 和 net 的 pins](src/placerBase.cpp#L978)。映射表用于查找，并没有在这里消除 vector 的顺序。分 region 的 [`PlacerBase::placeInsts_`](src/placerBase.cpp#L1132)也保持筛选后的公共实例顺序。

### 3.2 固定 seed 不等于按实例稳定的扰动

[`NesterovBaseCommon`](src/nesterovBase.cpp#L1245)从上述容器构建 GCell/GNet/GPin；[`NesterovBase` 构造](src/nesterovBase.cpp#L2216)按 `pb_->placeInsts()` 顺序为每个实例消耗随机数，调用 [`calculatePlacementPerturbationOffset`](src/nesterovBase.cpp#L2146)。

随机发生器用 `initialPlacePerturbationSeed` 初始化，但它是一条共享随机流：调换 A、B 的遍历位置，会调换分配给 A、B 的随机位移。即便 initial_place 的结果恰好相同，后续起点仍可不同。当前[默认扰动距离为自动值 `-1`、seed 为 1](include/gpl/Replace.h#L91)，扰动通常启用；`-perturb_dist 0` 才跳过该扰动。

这一点属于**公共 Nesterov 初始化**，不是 `InitialPlace::placeInstsInitialPositions()` 的随机性；后者本身没有随机采样。

### 3.3 梯度的浮点归约

CPU [`updateWireLengthForceWA_native`](src/nesterovBase.cpp#L1445)按 net 内 GPin 顺序累加指数和、坐标乘指数和；[`getWireLengthGradientWA`](src/nesterovBase.cpp#L1567)按 instance 的 GPin 顺序累加带 timing/custom weight 的梯度。即使坐标相同，仅调整 pin 顺序也可能产生 float 舍入差异。

所以跳过 initial_place、固定 seed、使用单线程，分别只排除部分来源，不能单独证明 timing/routability 完整流程对 ODB 顺序独立。线程调度与 ODB 顺序是两个不同变量，本研究不把它们混为同一原因。

另一个输入边界是 [`getRows()` 中首个非 PAD row 的 site](src/placerBase.cpp#L765)会被用作公共 site 尺寸。若研究任意 ODB 接口且允许混合 site，还需单独控制 row 顺序；普通单 site 设计没有这个差异来源。

## 4. initial_place

### 4.1 pin 并列极值改变 B2B 建模：直接且非数值噪声

调用链为 [`doBicgstabPlace`](src/initialPlace.cpp#L51) → `placeInstsInitialPositions` → `setPlaceInstExtId` → 每轮 `updatePinInfo` → `createSparseMatrix` → `cpuSparseSolve` → `updateCoordi`。

[`updatePinInfo`](src/initialPlace.cpp#L203)使用严格的 `>` / `<` 更新最小/最大 pin：

```cpp
if (lx > pin->cx()) { /* 将当前 pin 标为唯一 MinPinX */ }
if (ux < pin->cx()) { /* 将当前 pin 标为唯一 MaxPinX */ }
```

当多个 pin 的 X 相同，最早遇到者保留标记；Y 同理。随后 [`createSparseMatrix`](src/initialPlace.cpp#L309)只对“至少一端是该方向 min/max”的 pin 对建边。换一个同坐标极值代表，就会换一组边，而不只是交换矩阵行列编号。多个未放置实例从 core/region 中心出发时也可能出现这种并列，不能假设所有极值天然唯一。

**构造例**：同一 net 的 pin 为 `A=0, B=0, C=10, D=20`。D 为固定 terminal；A/B/C movable，零 pin offset。只将序列 `A,B,C,D` 改为 `B,A,C,D`：

| pin 顺序 | min-X / max-X | 该 net 的 B2B 边 |
| --- | --- | --- |
| A,B,C,D | A / D | AB、AC、AD、BD、CD |
| B,A,C,D | B / D | AB、BC、BD、AD、CD |

增加固定 terminal `L=-10`、`R=30`，以及两个二 pin net `A-L`、`B-R`，所有坐标按 `minDiffLength` 归一化，使用生产实现的 `1/(fanout-1)/max(distance, minDiffLength)` 权重形式。公共 `netWeightScale` 在求解中抵消。精确有理数求解得到：

| 顺序 | 一轮求解后的 A/B/C 坐标 | 三个 net 的总 X-HPWL |
| --- | --- | --- |
| A,B,C,D | 285/61，460/61，1505/122 | 3200/61 = 52.459016393 |
| B,A,C,D | 130/31，475/62，1715/124 | 3245/62 = 52.338709677 |

此例证明**相同对象、连接、坐标，仅 pin 顺序不同，即可改变方程、解和模型 HPWL**，与浮点误差无关。可整体平移并按默认 `minDiffLength=1500` 缩放至正坐标。它验证的是一轮 B2B 模型，未包含生产代码的坐标整数截断、完整外层迭代或后续 Nesterov，因此这些数字不是 OpenROAD 完整布局的测量值。

### 4.2 net 顺序：浮点装配

`createSparseMatrix` 按 `pbc_->getNets()` 和 pin 对顺序产生 `Eigen::Triplet<float>`，并以 `+=` 累加 float RHS；[最后 `setFromTriplets`](src/initialPlace.cpp#L481)合并重复矩阵元素。精确算术下 net 贡献可任意重排，但 float 加法不满足结合律，当前逐项 RHS 累加已不能保证结果对置换不变；不能仅因稀疏矩阵最后按索引组织就认为装配顺序无影响。

### 4.3 instance 顺序：编号本身不是离散模型变化

[`setPlaceInstExtId`](src/initialPlace.cpp#L183)以 movable instance 的 vector 位置作为变量编号。若 pin 极值选择及其他数据完全一致，在精确算术下重新编号只是 `A' = P A Pᵀ, b' = P b`，按实例还原后解应相同。

实际 [`cpuSparseSolve`](src/solver.cpp#L11)使用 float BiCGSTAB、IdentityPreconditioner 和有限 `maxSolverIter`；变量顺序可改变点积/稀疏乘法舍入。残差、[外层停止条件](src/initialPlace.cpp#L115)及[坐标转 int](src/initialPlace.cpp#L485)提供了进一步分岔的机会。这是数值敏感路径，不能与 4.1 的建模差异等量齐观。

**结论：initial_place 对 ODB 顺序敏感；最强的直接证据是 `dbNet::getITerms()` 导致的极值 tie-breaking。** 若没有并列极值，并固定几何与初始位置，剩余 net/instance 顺序影响主要是有限精度层面。

## 5. timing_driven

### 5.1 给定 slack 时，加权内核对顺序独立

[`runTimingDriven`](src/nesterovPlace.cpp#L375)根据 overflow 决定触发时机，写回坐标，再调用 [`TimingBase::executeTimingDriven`](src/timingBase.cpp#L126)。后者调用 resizer 获取 slack，然后确定权重。

[`Resizer::resizeWorstSlackNets`](../rsz/src/Resizer.cc#L3340)从 slack map 收集 net 后，明确按 `(slack, dbNet ID)` 排序，再取最差的百分比。因此 map/ODB 的收集顺序不直接决定排名。

`TimingBase` 取该序列的首尾 slack 作为 `s_min`、`s_cutoff`，随后遍历**全部 GNet**，按各自 dbNet 查询 slack：

```text
有有效 slack 且 s < s_cutoff，且 s_min != s_cutoff 时：
    weight = 1 + (weight_max - 1) * (s_cutoff - s) / (s_cutoff - s_min)
其他情况：weight = 1
```

所以它不是“按 ODB 顺序把前 K 个 GNet 加权”。给定相同有效 net 集合和逐 net slack，置换 `getGNets()` 只改变赋值先后，不改变各 net 的权重。

即使重建数据库导致同 slack net 的 ID 不同，**单就此内核**，cutoff 处同分 net 的选择也不会改变权重：相同 slack 给出相同阈值，且等于 cutoff 的 net 不进入严格 `<` 分支。`s_min == s_cutoff` 时权重为 1，也不会因选中哪一个同分 net 而加权不同。

### 5.2 上游已有稳定化，不能把有序处理一概判成敏感

[`findResizeSlacks`](../rsz/src/Resizer.cc#L3226)会估计寄生、执行 `repairDesign`；placement 寄生路径还会 `fullyRebuffer`，可选执行 setup repair，最后收集 slack。virtual 模式回滚 netlist，非 virtual 模式保留修改。**回滚 netlist 不会抹掉本次用于加权的 slack 计算结果。**

其中 [`dbSta::levelizedDrvrVertices`](../dbSta/src/dbSta.cc#L347)明确按 `(level, vertex name)` 排序；[`RepairDesign`](../rsz/src/RepairDesign.cc#L346)和 [`fullyRebuffer`](../rsz/src/Rebuffer.cc#L2113)使用这个序列。因此“driver 来自数据库，所以修复 driver 顺序必然随 `getInsts()` 改变”的说法不成立。

[`EstimateParasitics::makeSteinerTree`](../est/src/EstimateParasitics.cpp#L1569)也按 `(x,y)` 排序 connected pins，消除了坐标互异时的一部分输入顺序影响。不过同坐标 pin 没有名称第二排序键，不能据此声称整条时序链路已经完全规范化。本文不将这一点单独当作已证明的最终 slack 差异。

### 5.3 仍然存在由 ODB pin 顺序进入 STA 的数值路径

对普通 flat dbNet，有如下可直接追踪的路径：

```text
dbNet::getITerms() / getBTerms()
  → dbNetwork::visitConnectedPins（按接口顺序调用 visitor）
  → Sdc::netCaps / FindNetCaps / Sdc::pinCaps
  → float pin_cap += portCapacitance(...)
  → GraphDelayCalc 的负载电容、delay/slew
  → resizer 修复及 resizeNetSlack
  → TimingBase 的权重
```

依据：[dbNetwork flat net visitor](../dbSta/src/dbNetwork.cc#L2711)、[OpenSTA `Sdc::netCaps` / `pinCaps`](../sta/sdc/Sdc.cc#L3253)、[`GraphDelayCalc::netCaps`](../sta/dcalc/GraphDelayCalc.cc#L1436)。这里确实将 ODB pin 顺序用于浮点求和，没有先按名称规范化求和顺序。不同电容的归约可能改变末位数值，再影响阈值比较、修复或 slack；不能据此推断常规库、常规设计上一定会有可见 WNS 差异。

此外，第 3 节的初始坐标、随机扰动、WA 梯度会直接影响 timing-driven 的几何输入与 overflow 触发点。若要测 timing 功能自身，必须固定这些输入，不能只比较两次完整 `global_placement -timing_driven` 后便把所有差异归给权重排序。

**结论：完整 timing_driven 不能保证 QoR 对 ODB 顺序不变；但固定 slack 后的 net reweight 内核具有顺序独立性，这是与 initial_place 的关键区别。**

## 6. routability_driven

### 6.1 默认 RUDY 路径：按 net 顺序累加 float

当前 [`routabilityUseRudy` 默认为 true](include/gpl/Replace.h#L70)。[`RouteBase::routability`](src/routeBase.cpp#L855) → [`calculateRudyTiles`](src/routeBase.cpp#L639) → [`Rudy::calculateRudy`](../grt/src/Rudy.cpp#L120)。未提供 selection 时，RUDY 直接遍历 `block_->getNets()`，将每个 net 的 demand 加入相交 tile。

[`Tile::addDemand`](../grt/src/Rudy.cpp#L199)为 `demand_ += demand`，存储与贡献均为 float。相同 net 集合按不同顺序累加，tile demand 不保证位级一致。需求会进入 [congestion ratio 与 inflation](src/routeBase.cpp#L628)、[RC 统计](src/routeBase.cpp#L1142)，再影响 [target RC 停止、最优解保存/回退和面积预算](src/routeBase.cpp#L855)。因此即使误差最初很小，也存在跨阈值后改变布局轨迹的机制；实际幅度需要设计级实验确定。

需要同时指出一个不敏感的子步骤：[`dbNet::getTermBBox`](../odb/src/db/dbNet.cpp#L2067)合并所有 ITerm 坐标及已放置 BPin 中心，是整数 min/max。**固定几何时，仅改变同一 net 的 ITerm/BTerm 次序，不会改变这个 bbox，也就不会改变该 net 的 RUDY 几何贡献。** 默认 RUDY 的直接数值敏感入口主要是 net 间累加顺序，而不是“bbox 的端点选了不同 pin”。布局器和 RUDY 对 port 的几何表示也不能混淆：GPL 的 [`Pin::updateCoordi(dbBTerm*)`](src/placerBase.cpp#L505)使用整个 BTerm bbox 中心，并非取第一个 BPin。

### 6.2 可选拥塞 net 加权：同分 top-K 可能产生离散差异

当前代码还包含 [`weightCongestedNets`](src/routeBase.cpp#L474)。[`getNetDemandInTiles`](../grt/src/Rudy.cpp#L158)按 `block_->getNets()` 顺序输出 `(net,demand)`；GPL 只按 demand 降序 `std::ranges::sort`，没有 net 名称或 ID 的 tie-break，再取前 K 个。

关键边界：

- **默认 `routabilityNetWeightMax=1.0`，该路径会提前返回，不参与默认结果。** 需显式启用大于 1 的最大权重，且选择百分比大于 0。
- 若 `worst > cutoff`，cutoff 同分项的权重算为 1；只交换这些边界同分项，未必改变实际权重。
- 若 `worst == cutoff`，入选的 K 个 net 都赋予 `netWeightMax`。当最高 demand 的同分组跨越 K 边界时，缺少第二排序键会使不同 net 得到大于 1 的权重。

例如 a/b/c/d 的 demand 都是 3，取 25%，最大权重 5：本机 C++ 标准库对正序输入选择 a，逆序选择 d。这证明该同分分支可改变实际加权 net 身份。标准并不承诺 `std::sort` 保留同分输入顺序，本例也不依赖这种跨平台承诺：问题在于没有规定一个与输入排列无关的同分结果。

[`applyNetWeights`](src/routeBase.cpp#L547)随后按 dbNet 查表并取最大权重，不是新的按 map 遍历顺序分配权重的问题。

### 6.3 GRT 路径：net 总表排序不代表 pin 输入也规范化

启用 `-routability_use_grt` 时，[`getGrtResult`](src/routeBase.cpp#L313)调用 global router，允许拥塞、将 congestion iterations 和 critical-net percentage 设为 0。

[`GlobalRouter::findNets`](../grt/src/GlobalRouter.cpp#L5049)对 clock / non-clock 两组分别按名称排序，clock 在前。因此，**仅将 `getNets()` 总表顺序打乱，并不能直接推断 FastRoute 的 net 路由先后改变**。固定配置下，非零 routing seed 的 shuffle 也作用在这个已排序序列上。

但 [`updateNetPins`](../grt/src/GlobalRouter.cpp#L5220) → [`makeItermPins`](../grt/src/GlobalRouter.cpp#L5286) / `makeBtermPins` 仍按 ODB pin 顺序追加。随后 [`findFastRoutePins`](../grt/src/GlobalRouter.cpp#L1777)按首次遇到的 `(grid x, grid y, layer)` 去重，**仅在 pin 非重复时才更新 driver 的 `root_idx`**。

反例：S0 在网格位置 0，S1 和 D 在网格位置 1，D 是 driver，三者同层：

| ODB pin 顺序 | 去重后的网格位置集合 | root 指向的位置 |
| --- | --- | --- |
| S0, S1, D | {0,1} | 0；D 因重复被跳过，没有更新 root |
| S0, D, S1 | {0,1} | 1 |

这不是 root index 随数组置换而变化：**根对应的物理位置变了**。该值由 [`makeFastrouteNet`](../grt/src/GlobalRouter.cpp#L1961)传入 FastRoute；[正 alpha 的 Steiner 构造](../grt/src/fastroute/src/RSMT.cpp#L568)明确使用 `getDriverIdx()`。所以存在配置相关的离散路由输入敏感点。不能由此宣称默认 alpha 下每次 root 改变都必然改变拥塞，更不能宣称所有 GRT 后端均有相同表现。

### 6.4 inflation 循环本身不是“先处理先占面积”

固定 congestion map、GCell 坐标与尺寸后，[逐 cell inflation](src/routeBase.cpp#L943)根据 cell 所在 tile 的比例独立计算，实际面积差以整数累计；没有逐 cell 消耗剩余额度、先到先得的逻辑。[`scaleInflationToBudget`](src/routeBase.cpp#L582)先求总需求，再统一缩放 tile 比例，其 double 总和仍有数值归约的顺序风险，但不应误称为顺序贪心膨胀。

**结论：routability_driven 存在顺序敏感路径；默认 RUDY 以数值累加为主要直接原因，可选 net reweight 和 GRT pin 去重还存在离散路径。**

## 7. 已完成验证与结论边界

本次未修改生产代码、未修改 OpenSTA、未重建或重排真实设计数据库。执行了独立机制验证，代码见附录：

| 验证 | 实际结果 | 能证明什么 |
| --- | --- | --- |
| 有理数 B2B 装配/求解，交换 A/B | 方程边集、解及模型 HPWL 不同，见 4.1 | 初始布局 pin tie-breaking 的离散敏感性，不依赖浮点 |
| 固定 slack 的 4-net timing 内核 | 24 种输入排列权重均为 `{1:5, 2:1, 3:1, 4:1}` | 此内核对遍历次序独立；不是对整个 timing 流程的测试 |
| binary32 的逐项累加 | `1 + 2^-24 + 2^-24 = 1.0`；先加两个小量再加 1 得 `1.0000001192092896` | float 非结合；对应源码中的累加形式，不是实际 RUDY tile/库电容测量 |
| C++ 同分 demand 排序、取一个 net | 正序选 a，逆序选 d，权重均为 5 | RUDY 可选同分 top-K 可改变实际加权对象 |
| C++ GRT 去重/root 规则 | 同一网格 pin 集合得到 root 位置 0 或 1 | pin 遍历可改变物理 root；不是完整路由 QoR 测试 |

执行环境：Python 3.12.3；GCC 15.2.0，`-std=c++20 -O2`。这些小程序复现的是生产源码中的相关规则，**没有链接或执行当前生产函数**。

现有 `build/bin/openroad -version` 返回 `6f17adec9b9876ba003d876d5104c8142088d634`，与本次源码基线不同，故没有将该二进制的布局输出作为当前版本的端到端证据。本文未给出真实设计的最终 HPWL/WNS/TNS/RC 差值、变化概率或显著性结论。

## 8. 后续设计级验证方案

若需要量化 QoR 差异，应使用与源码基线一致的二进制，针对每个接口单独做原序、逆序和多组固定 seed 的 shuffle，且保留原始对象 ID。适合在 GPL 消费边界复制为 vector 后置换，避免全局修改迭代器造成无关模块、输出顺序和约束同时变化。

| 隔离目标 | 必须固定的输入 | 优先比较的中间状态 | 最终指标 |
| --- | --- | --- | --- |
| initial_place | 同一 ODB、初始坐标、参数与线程；不进入 Nesterov | 每轮 min/max pin 名称、按实例名称还原后的 A/b、残差、坐标 | initial HPWL；再单独观察后续 GPL HPWL |
| timing 加权内核 | 固定逐 net slack map 与有效集合 | 排名阈值、按 net 名称的 weight | 权重应相同；无需完整布局 |
| 完整 timing 功能 | 同一 placement snapshot、关闭扰动；恢复完整 resizer/STA 状态 | pin capacitance、逐 net slack、修复集合、权重与触发点 | 相同寄生提取条件下的 WNS/TNS、面积、HPWL |
| RUDY | 同一坐标与路由资源；只置换其 net 消费序列 | 逐 tile demand、actionable mask、RC、inflation、可选 net weight | RC、overflow、HPWL；完整 GPL 后重新评估 |
| GRT / FastRoute | 同一坐标/资源/alpha/seed；分别置换 net 与 pin 序列 | 名称排序后的 net 序列、每 net 网格 pin、root 位置、usage map | 路由拥塞及最终 GPL QoR |

每个变体开始前按名称校验连接集合、master、几何和初始坐标相同；报告结果时分开统计“坐标/中间状态变化”和“最终 QoR 变化”。单线程、同一构建重复运行先验证基线稳定，再独立测试多线程。微小数值差异与阈值导致的离散分岔应分别记录。

## 9. 若目标是建立顺序不变性

以下是依据上述机制给出的改进方向，**本次没有实施这些修改**：

1. GPL 收集 instance/net/pin 时规定完整语义排序键；跨 ODB 重建也要稳定时，优先用完整对象名称及 pin 名称，不能只依赖创建 ID。单纯 stable_sort 只保留原序，不会解决同分敏感性。
2. initial_place 为并列极值指定稳定代表；保持 B2B 模型不变的前提下再处理矩阵/RHS 累加顺序。
3. 将 Nesterov 扰动绑定到稳定实例身份与 seed，或按规范顺序消费随机流；固定 seed 本身不够。
4. 保留 timing 已有排序与阈值加权语义；对 STA 的电容归约、重合点映射等按实际实验结果逐项处理。若需修改 `src/sta/`，须先按项目要求取得授权并考虑上游维护。
5. RUDY 规范 net 累加顺序；可辅以更高精度/补偿求和降低误差，但这不等同于数学意义的任意置换严格不变。可选拥塞加权增加完整 tie-break，或定义统一处理边界同分组的语义。
6. FastRoute pin 去重时独立维护 driver 到已有网格点的映射，避免 driver 是否首个出现改变 root。其余 pin 输入是否也需规范化，应继续结合实际 Steiner 分支验证。

## 10. 三个 feature 全关闭后，基础 GPL 是否仍敏感？

**仍存在顺序敏感路径。** 关闭的是 B2B 初始求解、时序反馈和可布线性反馈，基础 Nesterov 的初始化、WA 线长梯度、密度惩罚和步长求解仍在运行。此结论仍指存在潜在 QoR 传播路径，未新增真实设计的最终 QoR 测量。

### 10.1 对应配置及容易混淆的选项

普通新调用中，可使用以下配置表达三个 feature 全关闭：

```tcl
global_placement -skip_initial_place
```

`timingDrivenMode`、`routabilityDrivenMode` 的默认值均为 false，不添加相应 flag 即可。[`-skip_initial_place` 的实现](src/replace.i#L40)是设置 `initialPlaceMaxIter=0`，跳过 B2B 外层求解；它不会设置 Nesterov 的 `initialPlacePerturbationDist=0`。当前 [`doBicgstabPlace`](src/initialPlace.cpp#L51)仍先执行初始坐标设置，再进入零次求解循环，因此也不应把这个选项理解为彻底跳过所有坐标初始化。

### 10.2 默认仍有按实例顺序分配的随机扰动

第 3.2 节的扰动属于 Nesterov，且与 timing/routability 的开关无关。在同一 seed 下，若随机序列为 `r1,r2,...`：

```text
实例遍历 A,B → A 使用 r1，B 使用 r2
实例遍历 B,A → B 使用 r1，A 使用 r2
```

所以只改变 `getInsts()` 顺序，即可改变按实例身份比较的优化起点。这是默认基础 GPL 中一个明确的顺序依赖，不需要借助浮点误差来解释。

### 10.3 再关闭扰动，也没有得到任意置换不变性

可以进一步使用：

```tcl
global_placement -skip_initial_place -perturb_dist 0
```

这排除了上述随机位移分配，但以下 CPU 路径仍存在：

| ODB 顺序来源 | 基础 Nesterov 中的消费点 | 影响路径 |
| --- | --- | --- |
| `net->getITerms()` / `getBTerms()` | [`updateWireLengthForceWA_native`](src/nesterovBase.cpp#L1445) 按 GPin 顺序累加 float 指数和、坐标加权和 | 改变 WA 梯度的数值结果 |
| `inst->getITerms()` | [`getWireLengthGradientWA`](src/nesterovBase.cpp#L1567) 累加同一 cell 上各 pin 的梯度 | 即使 timing/custom weight 都为 1，也仍有归约顺序差异 |
| `block->getInsts()` | [`updateGradients` 的串行归约](src/nesterovBase.cpp#L4062)按 GCell vector 顺序求 `wireLengthGradSum_`、`densityGradSum_` | 两者用于[初始化 density penalty](src/nesterovBase.cpp#L3830)，可影响所有 cell 的后续更新 |
| `block->getInsts()` | [`getDistance`](src/nesterovBase.cpp#L6006)按坐标 vector 顺序累加 float 平方距离 | [`getStepLength`](src/nesterovBase.cpp#L3860)用坐标距离/梯度距离决定步长，进而影响后续坐标 |

代码中的“Serial reduce for determinism”保证的是**给定固定 vector 顺序时**，归约不随并行执行先后改变。它并没有将归约顺序规范为对象名称顺序，所以不等于对 ODB 返回顺序独立。使用单线程也不会消除表内这些逐项 float 累加路径。

### 10.4 哪些地方不应误判为敏感？

在 CPU-only 构建中，[`scatterDensityAreaInPlace`](src/nesterovBase.cpp#L975)先对每个 cell 的 bin 面积贡献分别转为 `int64_t`，然后原子相加。固定 cell 几何、尺寸、density scale 和 bin 网格，且无整数溢出时，其结果是固定整数多重集的和，**不受 cell 遍历顺序影响**。因此不能将这一步笼统说成浮点 atomic 导致的顺序敏感；`ENABLE_GPU` 构建另有分支，不包含在这条 CPU-only 判断中。

此外，在普通单 region、无额外可选功能、pin 顺序固定的基础流程中，仅置换 `block->getNets()` 总表，不应自动解释为 WA 梯度改变：逐 net 的 WA 累加发生在各自 net 内，和 RUDY 的跨 net tile 累加不同。本文确认基础 GPL 整体存在敏感路径，**并不声称每个 ODB 接口单独置换都会改变结果**。

综上，三 feature 全关闭仍不足以保证 QoR 顺序不变；再加 `-perturb_dist 0` 可排除一个明确来源，剩余 pin/GCell 浮点归约还需要规范化或设计级验证。

## 附录：机制验证复现代码

保存下列两个代码块为 `/tmp/gpl_order_check.py` 与 `/tmp/gpl_order_check.cpp`，执行：

```bash
python3 /tmp/gpl_order_check.py
g++ -std=c++20 -O2 /tmp/gpl_order_check.cpp -o /tmp/gpl_order_check
/tmp/gpl_order_check
```

### Python：精确 B2B 反例、float32 累加、固定 slack 的加权

```python
from fractions import Fraction as F
from itertools import permutations
import struct


def solve(a, b):
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    n = len(b)
    for i in range(n):
        p = next(j for j in range(i, n) if m[j][i])
        m[i], m[p] = m[p], m[i]
        d = m[i][i]
        m[i] = [v / d for v in m[i]]
        for j in range(n):
            if j != i:
                d = m[j][i]
                m[j] = [x - d * y for x, y in zip(m[j], m[i])]
    return [row[-1] for row in m]


def initial(order):
    # Coordinates normalized by minDiffLength; zero pin offsets.
    x = dict(A=F(0), B=F(0), C=F(10), D=F(20), L=F(-10), R=F(30))
    movable = 'ABC'
    a = [[F(0) for _ in movable] for _ in movable]
    b = [F(0) for _ in movable]
    edges = []
    for pins in [order, 'AL', 'BR']:
        lo = min(pins, key=x.get)  # Strict comparisons retain first tie.
        hi = max(pins, key=x.get)
        for i, p in enumerate(pins):
            for q in pins[:i]:
                if p not in (lo, hi) and q not in (lo, hi):
                    continue
                w = F(1, len(pins) - 1) / max(abs(x[p] - x[q]), F(1))
                edges.append(''.join(sorted(p + q)))
                for u, v in [(p, q), (q, p)]:
                    if u in movable:
                        j = movable.index(u)
                        a[j][j] += w
                        if v in movable:
                            a[j][movable.index(v)] -= w
                        else:
                            b[j] += w * x[v]
    loc = dict(x, **dict(zip(movable, solve(a, b))))
    hpwl = sum(max(loc[p] for p in net) - min(loc[p] for p in net)
               for net in ['ABCD', 'AL', 'BR'])
    print(f'initial {order}: edges={sorted(edges)}, '
          f'x={[str(loc[p]) for p in movable]}, HPWL={hpwl} ({float(hpwl):.9f})')
    return hpwl


assert initial('ABCD') != initial('BACD')


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def accumulate(xs):
    s = f32(0)
    for x in xs:
        s = f32(s + f32(x))
    return s


u = 2 ** -24
print('float32 sum:', accumulate([1, u, u]), accumulate([u, u, 1]))
assert accumulate([1, u, u]) != accumulate([u, u, 1])


def timing(order):
    slacks = {1: -3, 2: -2, 3: -2, 4: 1}
    ranked = sorted(order, key=lambda n: (slacks[n], n))[:3]
    lo, hi = slacks[ranked[0]], slacks[ranked[-1]]
    return {n: 1 + 4 * (hi - s) / (hi - lo) if s < hi and lo != hi else 1
            for n, s in slacks.items()}


weights = timing([1, 2, 3, 4])
assert all(timing(p) == weights for p in permutations([1, 2, 3, 4]))
print('timing fixed slacks: 24 permutations identical:', weights)
```

### C++：同分拥塞 net 选择与重合 pin 的 root

```cpp
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

int main()
{
  for (std::vector<std::pair<char, float>> demand : {
           std::vector<std::pair<char, float>>{{'a', 3}, {'b', 3}, {'c', 3}, {'d', 3}},
           std::vector<std::pair<char, float>>{{'d', 3}, {'c', 3}, {'b', 3}, {'a', 3}}}) {
    std::ranges::sort(demand, std::greater<>{}, [](const auto& e) { return e.second; });
    std::cout << "RUDY top 25%, worst == cutoff: " << demand[0].first << " weight=5\n";
  }
  struct Pin { int x; bool driver; };
  for (std::vector<Pin> pins : {
           std::vector<Pin>{{0, false}, {1, false}, {1, true}},
           std::vector<Pin>{{0, false}, {1, true}, {1, false}}}) {
    std::vector<int> positions;
    int root = 0;
    for (const Pin& pin : pins) {
      const bool duplicated = std::ranges::find(positions, pin.x) != positions.end();
      if (!duplicated) {
        positions.push_back(pin.x);
        if (pin.driver) { root = positions.size() - 1; }
      }
    }
    std::cout << "GRT same grid pin set: root position=" << positions[root] << '\n';
  }
}
```
