# 理解 dbSite：芯片上的"停车位"

---

## 1. 一句话定义

**`dbSite` 是芯片上标准单元（standard cell）的最小放置格子——它定义了单元在 X 和 Y 方向上必须对齐的网格间距，是所有布局操作的"最小刻度"。**

---

## 2. 核心机制（300 字以内）

芯片设计中有成千上万个标准单元（与门、或门、触发器等），它们不能随意摆在芯片上——必须像拼积木一样整齐排列。

`dbSite` 解决了"积木该以多大间距排列"的问题。它定义了两个核心数字和一个类别标记：

- **宽度（width）**：水平方向上，相邻两个单元原点之间的步进距离。`core 宽度 ÷ site 宽度 = 每行可放的单元数`。
- **高度（height）**：垂直方向上，一行单元与下一行之间的间距。`core 高度 ÷ site 高度 = 可放的行数`。
- **类别（class）**：`CORE` 表示核心区域放置格子，`PAD` 表示 I/O 焊盘区域格子。IFP 主要处理 `CORE` 类型的 site。

此外还有三个**对称性标记**（`x_symmetry`、`y_symmetry`、`R90_symmetry`），告诉工具放在这个 site 上的单元是否可以水平翻转、垂直翻转或旋转 90°。这直接影响 Row 的朝向编排——偶数行正向（R0），奇数行翻转（MX），以保证相邻行的电源/地轨共享。

> **术语解释**：DBU（Database Unit，数据库单位）是芯片内部坐标的精度单位，通常为 1 纳米或更小。Site 的宽高都以 DBU 存储，展示时通过 `dbuToMicrons()` 换算为微米。

---

## 3. 生活化类比

### 类比一：停车场的停车位

| 停车场概念 | 芯片概念 | 对应关系 |
|-----------|---------|---------|
| 一个标准停车位（2.5m × 5m） | `dbSite`（如 0.5μm × 2μm） | 定义了车辆能停放的**最小空间格子** |
| 一排停车位 | `dbRow`（一行 site） | 由同一个 site 沿水平方向重复铺开 |
| 一辆车 | 一个标准单元（与门、触发器等） | 必须停在格子内，不能跨线 |
| 停车场画线规则 | LEF 文件中的 SITE 定义 | 规定每个车位多大、朝向如何 |

你不能把车停在线外面——同样，标准单元也必须对齐到 site 网格。site 的宽度就是相邻两个"车位"的中心间距。一辆卡车（大单元）可能占多个车位（多倍 site 宽度），但它的起始位置仍然要对齐到车位线。

### 类比二：方格纸/坐标纸

想象你有一张方格纸，每个小方格是 0.5μm × 2μm。你要在这张纸上画矩形代表标准单元：

- 每个矩形必须对齐到方格边界——这就是 site 的约束。矩形可以是 1 格宽、2 格宽甚至 10 格宽，但左边界必须落在格子线上。
- 一行格子 = 一个 `dbRow`，所有格子在水平方向对齐。
- 混合高度 site（hybrid row pattern）就像方格纸的**某些行高、某些行矮**，按照一定模式重复（比如"高-矮-高-矮"），对应芯片上混合使用不同高度的单元库。

---

## 4. 易混点

### 易混点 1：`dbSite` 不是 `dbRow`

这是最常见的新手误解。

| | `dbSite` | `dbRow` |
|---|---|---|
| 是什么 | 一个**模板/定义**——"标准停车位的尺寸规格" | 一个**实际存在的东西**——"芯片上第 3 排、从坐标 (100, 200) 开始的 50 个停车位" |
| 来源 | 来自工艺库 LEF 文件的 `SITE` 关键字 | 由工具（如 IFP 的 `makeRows`）在芯片上创建 |
| 数量 | 一个工艺库通常只有几个 site 定义 | 一个芯片上可能有成千上万个 row |
| 关系 | Row 引用一个 site："我这一行用 core site" | Row 是 site 的实例化 |

一句话区分：**site 是规格说明书，row 是按说明书造出来的实物。**

### 易混点 2：site 的 class（`CORE` vs `PAD`）描述的是**区域类型**，不是**单元类型**

`dbSiteClass::CORE` 的意思是"这个 site 用在芯片核心区域"，而不是"这个 site 只能放 core 类型的单元"。同理，`dbSiteClass::PAD` 表示"这个 site 用在 I/O 焊盘区域"。

一个标准单元（如 NAND2）的 `dbMaster::getSite()` 返回的 site，它的 class 通常是 `CORE`，因为标准单元放在核心区域。一个 I/O 焊盘单元的 site 则是 `PAD`。

IFP 在处理电压域和 Track 时用 `getClass()` 过滤掉 PAD site 的行——PAD 区域的 Row 不参与核心区域的 Row 切分和 Track 计算。

### 易混点 3：site 的 width 是**最小步进距离**，不是**单元的固定宽度**

新手常误以为"一个 site 的宽度 = 一个标准单元的宽度"，因此"每行 site 数 = 每行单元数"。实际上：

- 一个 site 宽度（如 0.5μm）是放置单元的**最小水平步进单位**。
- 一个单元可能占 1 个 site 宽度（小与门），也可能占 4 个 site 宽度（大触发器），甚至几十个 site 宽度（宏单元）。
- `dbMaster::getWidth()` 永远是 site 宽度的整数倍——单元设计时就保证了对齐。

所以在计算"一行能放几个单元"时，你不能只看 site 数量，还要看每个单元的实际宽度。**site 数量 = 一行能放的"最小单位格子"的数量，不是"单元"的数量。**

---

## 5. 自测题

### 题目 1：基础计算

一个 site 宽度 0.5μm、高度 2μm，Core 区域为 100μm × 80μm（左下角原点 (0, 0)）。

(a) Core 区域一行最多能放多少个 site？（不考虑阻塞）

(b) Core 区域最多能放多少行？（不考虑阻塞）

(c) 总共有多少个 site 位置？

> *思考完后再往下翻答案。*

---

**答案：**

(a) `100 / 0.5 = 200` 个 site/行

(b) `80 / 2 = 40` 行

(c) `200 × 40 = 8000` 个 site 位置

这就是 `dbRow::create` 中 `num_sites` 参数的计算方式——每行 site 数 = `core.dx() / site->getWidth()`。

---

---

### 题目 2：概念辨析

下面哪些说法是正确的？（多选）

A. 一个 dbRow 只能使用一种 dbSite。

B. 同一个 Core 区域内，所有 Row 的 site 高度必须一致。

C. `hasRowPattern()` 返回 true 的 site 是混合高度 site，它定义了一组子 site 的排列模式。

D. `dbSite::getSymmetryX()` 为 true 意味着放在该 site 上的单元支持沿 X 轴翻转。

> *思考完后再往下翻答案。*

---

**答案：**

**A ✓** — 一个 dbRow 引用一个 dbSite，整行统一使用同一种 site。

**B ✗** — 混合高度（hybrid）site 支持不同高度的子 site 在同一 Core 区域共存。例如 base site 定义了一个 "高-矮" 交替的 row pattern，那么高行和矮行的高度不同。但子 site 的高度必须是 base site 高度的整数倍，IFP 会检查这个约束（error code 54）。

**C ✓** — `hasRowPattern()` 返回 true 表示 `row_pattern_` 非空，这是一个混合高度 site。它的 `getRowPattern()` 返回 `[(site₁, orient₁), (site₂, orient₂), ...]`，逐行循环使用。

**D ✓** — `x_symmetry` 标记表示单元可以关于 X 轴翻转（即上下镜像），在 LEF 中对应 `SYMMETRY X`。

---

---

### 题目 3：site 与单元的尺寸关系

假设 site 宽度为 0.5μm。下面的说法哪一个才是正确的？

A. 每行最多能放 200 个单元。

B. 每行能放的单元数量取决于每个单元的实际宽度，一个占 4 个 site 宽的 DFF 比占 1 个 site 宽的 NAND2 占用更多位置。

> *思考完后再往下翻答案。*

---

**答案：**

**B 正确。**

site 数量（200）是格子数，不是单元数。就像停车场一排有 200 个车位，但一辆大巴可能占 4 个车位，所以实际停的车少于 200 辆。对应到芯片：`num_sites = core.dx() / site->getWidth()` 算出的是格子数，而 `dbInst` 的实际宽度（`dbMaster::getWidth()`）决定了每个单元占几格。IFP 中的 `designArea()` 正是通过累加每个 `dbInst` 的 `master->getWidth() * master->getHeight()` 来计算总设计面积的。

---

---

### 题目 4：混合高度 Row Pattern 的计算

假设一个混合高度 site `hybrid` 的 row pattern 为 `[(core_9t, R0), (core_12t, R180)]`，其中：

- `core_9t` 高度 1.8μm
- `core_12t` 高度 2.4μm

Core 区域高度为 12μm，从 y=0 开始。

(a) 整个 pattern 的循环周期高度是多少？

(b) 最多能完成几个完整的 pattern 循环？

(c) 最后一行的 site 是 `core_9t` 还是 `core_12t`？

> *思考完后再往下翻答案。*

---

**答案：**

(a) 周期高度 = `1.8 + 2.4 = 4.2μm`（一个 pattern 循环包含两行）。

(b) `12 / 4.2 = 2` 个完整循环（余 3.6μm，详见 (c)）。

(c) 2 个完整循环占 `2 × 4.2 = 8.4μm`，剩余 `12 - 8.4 = 3.6μm`。接下来的第 5 行是 `core_9t`（1.8μm），放得下（剩余变为 1.8μm）；第 6 行是 `core_12t`（2.4μm），但只剩 1.8μm 了，放不下。所以最后一行是 `core_9t`。总共 5 行。

这就是 `InitFloorplan::makeHybridRows()` 中的逻辑：逐行循环 pattern，`y + site->getHeight() > core.yMax()` 则停止。

---

---

### 题目 5：site 的查找路径

如果用户调用 `findSite("core_9t")`，IFP 会如何找到这个 site？以下哪个描述了正确的查找路径？

A. 直接在 `dbBlock` 中查找名为 "core_9t" 的 site。

B. 遍历 `dbBlock → dbDatabase → dbLib` 列表，在每个 `dbLib` 中调用 `findSite("core_9t")`。

C. 在 `dbTech` 中查找 site。

> *思考完后再往下翻答案。*

---

**答案：**

**B 正确。**

site 定义在 LEF 工艺库中，属于 `dbLib` 的子对象。查找路径为：

```
dbBlock → getDataBase() → getLibs() → 逐个 dbLib → findSite(name)
```

`InitFloorplan::findSite()` 的实现正是：

```cpp
for (dbLib* lib : block_->getDataBase()->getLibs()) {
    dbSite* site = lib->findSite(site_name);
    if (site) return site;
}
return nullptr;
```

**A 错误**——site 不属于 block，属于 lib。

**C 错误**——`dbTech` 管理工艺层（layer）和制造网格，不管理 site。

---

---

## 6. 复述挑战

> 试着用自己的话回答以下问题。如果你能流畅答完，说明你真的懂了。

**"假设你在给一个新同事讲解刚读完的 dbSite 代码。请用你自己的话回答：dbSite 是什么？它跟 dbRow 是什么关系？IFP 用它来干什么？如果一个工艺库同时有两种不同高度的 site（比如 1.8μm 和 3.6μm），IFP 怎么处理？"**

---

**参考答案（先自己尝试回答，再看答案）：**

**dbSite 是什么？**

dbSite 是工艺库（LEF）中定义的标准单元放置网格模板。它规定了两个关键数字：width（水平步进间距）和 height（行高）。芯片上所有标准单元都必须对齐到这个网格。它还携带 class 标记（CORE/PAD）区分核心区域和 I/O 区域，以及 symmetry 标记指示单元能否翻转/旋转。

**跟 dbRow 是什么关系？**

dbSite 是模板，dbRow 是实例。"site = 车位规格说明书（每个车位 2.5m×5m）"，"row = 停车场实际画出来的一排 50 个车位"。dbRow 引用一个 dbSite 来确定每个 site 的尺寸，然后沿水平方向重复该 site 形成一行。`dbRow::create()` 的参数 `num_sites` 就是这一行有多少个 site 位置。

**IFP 用它来干什么？**

IFP 的核心任务就是创建 Row 和 Track。创建 Row 时：
- 用 `site->getWidth()` 计算 `num_sites = core.dx() / site->getWidth()`
- 用 `site->getHeight()` 确定每行 Y 坐标的步进
- 用 `site->hasRowPattern()` 判断走普通路径还是混合高度路径
- Row 创建后，用 `block->setCoreArea(block->computeCoreArea())` 更新 Core 区域

**不同高度 site 共存怎么处理？**

有两种情况：

如果走**普通路径**（`makeUniformRows`），IFP 要求每个子 site 的高度必须是 base site 高度的整数倍（否则报 error 54）。然后为每个 site 各自生成一组行——比如 1.8μm 的 site 生成若干行 1.8μm 高的 row，3.6μm 的 site 也独立生成若干行 3.6μm 高的 row。不同 site 的行各自独立排列。

如果走**混合高度路径**（`makeHybridRows`，由 LEF 的 `ROWPATTERN` 定义），base site 的 `getRowPattern()` 返回固定的交替模式。IFP 按模式的循环顺序逐行创建，每行使用模式中指定的 site 和 orientation。

---

---

*提示：如果卡住了，回到第 2 节重读核心机制，再看第 4 节的易混点。*
