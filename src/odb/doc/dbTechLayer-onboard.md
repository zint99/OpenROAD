# dbTechLayer — onboard

> **适合读者**：刚开始接触 OpenROAD 的开发者，不需要预先了解物理设计（physical design）或 EDA 工具。

***

## 1. 一句话定义

**`dbTechLayer`** **是芯片制造工艺中"一层"的数据档案，记录了这一层的物理尺寸、电气特性、几何规则，以及它在整个层堆叠中的位置。**

如果芯片是一栋高楼，每一层（layer）就是楼里的一个楼层，而 `dbTechLayer` 就是那个楼层的"建筑说明书"。

***

## 2. 300 字以内解释

芯片不是画在纸上的 — 它是在硅片上通过光刻、蚀刻等工艺一层一层"盖"出来的。每一层都有自己的一套规则：线能画多宽、两根线至少隔多远、这一层的线只能横着走还是竖着走、电流通过时电阻多大……

`dbTechLayer` 就是用来存储这一切信息的"档案卡"。在代码中，它本质是 OpenROAD 数据库（ODB）中的一个对象，属于 `dbTech`（工艺库），记录着：

- **我是谁**：名字（如 "M1"、"M2"、"VIA1"）、编号、类型（走线层 ROUTING / 通孔层 CUT / 其他）。
- **我长什么样**：默认线宽（width）、最小间距（spacing）、走线方向（水平 HORIZONTAL / 垂直 VERTICAL）、网格步长（pitch）。
- **我的物理特性**：电阻（resistance）、电容（capacitance）、厚度（thickness）。
- **我和邻居的关系**：上层是谁（`getUpperLayer()`）、下层是谁（`getLowerLayer()`），形成一条层链表。
- **我的规则本**：几十种设计规则（design rules），比如"线宽大于 X 时，间距要变成 Y"（宽线规则）、"通孔（via）的金属必须比孔大多少"（包围规则）等。

所有这些数据来源于工艺文件（LEF/DEF），由 OpenROAD 在读取时填充到 `dbTechLayer` 中，供布线（routing）、布局（placement）等后续步骤查询使用。

***

## 3. 生活化类比

### 类比 1：城市规划中的道路层

想象你在设计一个多层立交桥系统：

| 立交桥概念           | dbTechLayer 对应                     | 说明                  |
| --------------- | ---------------------------------- | ------------------- |
| 每层桥面            | 一个 `dbTechLayer` 实例                | 每层桥面有独立的通行规则        |
| 桥面只能东西向或南北向行驶   | `direction`（HORIZONTAL / VERTICAL） | 走线层通常只沿一个方向布线，相邻层交替 |
| 车道宽度            | `width`                            | 这层上金属线的最小/默认宽度      |
| 车道间距            | `spacing`                          | 两条相邻线之间至少隔多远        |
| 上下层之间的匝道        | CUT 层 + via（通孔）                    | 通孔层连接上下两个走线层        |
| 桥面从地面起算的高度编号    | `number_` / `rlevel_`              | 层的编号和布线层序号          |
| 立柱的间距（决定了桥面的栅格） | `pitch`                            | 走线必须落在栅格上，保证对齐      |

### 类比 2：多层蛋糕的配方卡

假设你在做一个多层婚礼蛋糕：

| 蛋糕概念                           | dbTechLayer 对应         | 说明                         |
| ------------------------------ | ---------------------- | -------------------------- |
| 每一层蛋糕                          | 一个 `dbTechLayer`       | 每层有自己的配方（属性）               |
| 配方卡上写的"海绵蛋糕"或"奶油夹层"            | `type`（ROUTING / CUT）  | 走线层 = 蛋糕层，通孔层 = 奶油夹层（连接上下） |
| 每层的厚度                          | `thickness`            | 金属层的物理厚度                   |
| 配方卡编号："第 3 层"                  | `number_`              | 层在工艺中的序号                   |
| 配方卡上写"第 3 层的下面是第 2 层，上面是第 4 层" | `lower_` / `upper_` 指针 | 形成双向链表                     |
| 糖霜必须覆盖到蛋糕边缘外多少                 | `wire_extension`       | 线末端要延伸多少，确保连接可靠            |
| 配方卡上写"只准切成长方形"                 | `rect_only` 标志位        | 某些层只允许矩形图形                 |

***

## 4. 易混点

### 易混 1：`number_` vs `rlevel_`

- **`number_`**：这一层在工艺中出现的顺序号。所有类型的层都参与编号。比如：M1 是 1 号，VIA1 是 2 号，M2 是 3 号……
- **`rlevel_`**：**只有 ROUTING 类型**的层才有的"布线层序号"。比如 M1 是第 1 个布线层（rlevel=1），VIA1 是 CUT 类型所以没有布线层序号，M2 是第 2 个布线层（rlevel=2）。

**简单记法**：`number_` 是"大排队"，`rlevel_` 是"走线层的单独排队"。

### 易混 2：`dbTechLayer` 不是"屏幕上的图层"

如果你是前端开发者，可能会联想到 Photoshop 的图层或 HTML 的 z-index。但 `dbTechLayer` 是**数据模型**，不是渲染概念。它不直接画任何东西 — 它只是记录"这一层允许怎么画"。真正在芯片上画出来的金属线，是存储在 `dbWire`、`dbNet` 等对象中的。

### 易混 3：CUT 层不是"没有属性"

初学者容易认为只有 ROUTING 层重要，CUT 层是次要的。实际上 CUT 层（通孔层）同样承载了大量规则，比如：

- `cut_spacing_rules`：通孔之间至少隔多远
- `cut_enc_rules`（enclosure rules）：通孔周围的金属要比孔大多少（像邮票的边缘必须比邮票大一圈）
- `min_cut_rules`：至少打多少个孔

如果没有这些规则，芯片制造出来层与层之间的连接可能断开或短路。

***

## 5. 自测题

**题目 1**：下面哪个属性表示"这一层的走线方向"？

- A) `type`
- B) `direction`
- C) `pitch`
- D) `offset`

<details>
<summary>点击查看答案</summary>

**B)** **`direction`**。类型（type）说的是"这是走线层还是通孔层"，pitch 是栅格间距，offset 是栅格偏移。

</details>

***

**题目 2**：一个工艺有 M1、VIA1、M2、VIA2、M3 共 5 层。M3 的 `rlevel_` 是多少？

<details>
<summary>点击查看答案</summary>

**3**。只有 ROUTING 层参与 rlevel\_ 编号。M1 → rlevel=1，M2 → rlevel=2，M3 → rlevel=3。VIA 是 CUT 层，rlevel\_ 始终为 0。

</details>

***

**题目 3**：判断对错："`dbTechLayer` 自己会画线。"

<details>
<summary>点击查看答案</summary>

**错**。`dbTechLayer` 只存储"规则"（线宽、间距等），不存储实际的几何图形。实际画出来的线存在 `dbWire` 等对象中。把 `dbTechLayer` 想象成"交通规则手册"，它不负责造车。

</details>

***

**题目 4**：如果我想知道 M2 上面紧挨着哪一层，应该调用什么方法？

<details>
<summary>点击查看答案</summary>

**`getUpperLayer()`**。它返回指向上层 `dbTechLayer` 的指针。同理，`getLowerLayer()` 返回下一层。

</details>

***

**题目 5**：为什么 `dbTechLayer` 需要存储 `resistance_`（电阻）和 `capacitance_`（电容）？

<details>
<summary>点击查看答案</summary>

因为在布线（routing）完成后，工具需要计算信号从 A 点传到 B 点的延迟（delay）。延迟由电阻和电容共同决定。`dbTechLayer` 提供每单位长度的 R/C 值，布线器据此算出每条线的总延迟，确保芯片能在目标频率下正常工作。

</details>

***

## 6. 复述挑战

> **假装你要给一个刚入职的同事解释** **`dbTechLayer`。先自己说一遍，然后对照下面的要点检查。**

### 你应该能说清楚这些：

1. **它是什么**：`dbTechLayer` 是 OpenROAD 中表示芯片"某一层工艺信息"的数据对象。属于 `dbTech`（工艺库），一个工艺包含多个 `dbTechLayer`。
2. **它存什么**：这一层的名字、类型（走线层/通孔层/其他）、走线方向、默认线宽和间距、电阻电容、栅格信息，以及一大堆制造规则（间距表、包围规则、最小面积等）。
3. **层之间怎么关联**：通过 `lower_` / `upper_` 指针形成双向链表，`getLowerLayer()` 拿下一层，`getUpperLayer()` 拿上一层。最底层叫 bottom，最顶层叫 top。
4. **为什么要分这么多规则**：因为芯片制造是纳米级的物理过程。线太宽会浪费面积，太窄会断；间距太小会短路，太大又浪费。每一种规则都在约束一个具体的制造场景，`dbTechLayer` 把它们集中管理。
5. **它和"实际走线"的关系**：`dbTechLayer` 是"规则本"，告诉你这层能怎么画。`dbWire` 和 `dbNet` 才是实际画出来的线。布线器（router）会反复查 `dbTechLayer` 的规则，确保画出的每一条线都符合工艺要求。

***

*文档生成日期：2026-08-09*
*基于 OpenROAD* *`src/odb/src/db/dbTechLayer.h`* *和* *`dbTechLayer.cpp`* *源码编写*
