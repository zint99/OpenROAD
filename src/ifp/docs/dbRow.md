# dbRow 中文笔记：从放置网格到芯片上的一排位置

> 面向刚接触 OpenROAD / OpenDB 的新员工。读完本文，应能解释一个 row 的几何含义、看懂 DEF 的 `ROW` 语句，并沿源码找到它的创建和使用位置。本文以当前仓库实现为准。

## 1. dbRow 是什么？

**`dbRow` 描述一个 block 中，某种 site 从指定坐标开始、沿指定方向等间距重复的一排放置位置。** IFP（初始布局规划）用它建立标准单元的放置行，后续布局工具再把实际单元安排到这些位置上。

可以把 site 想成停车位的规格，把 row 想成停车场里画好的一排车位，把单元实例想成车辆。一辆大车可能占多个车位；同样，一个标准单元可能占多个 site 宽度。因此，**site 数量不是单元数量**。

| 对象 | 回答的问题 | 归属或关联 |
| --- | --- | --- |
| `dbSite` | 每个放置格子的尺寸和属性是什么？ | 属于 `dbLib` |
| `dbRow` | 从哪里开始、用什么 site、重复多少次？ | 属于 `dbBlock`，引用一个 `dbSite` |
| `dbMaster` | 某种单元的尺寸、引脚等库定义是什么？ | 属于 `dbLib`，可以关联 site |
| `dbInst` | 设计中实际用了哪个单元、放在哪里？ | 属于 `dbBlock`，引用 master |
| `dbTrackGrid` | 金属布线轨道在哪里？ | 属于 `dbBlock`，关联技术层 |

```text
dbLib                         dbBlock
  └─ dbSite ◄── 引用 ──────────── dbRow（放置位置）
  └─ dbMaster ◄─ 引用 ─────────── dbInst（实际单元）
```

`dbRow` 本身不保存“这一行放了哪些实例”的列表，也不会随着单元放入而减少 `site_count`。占用情况和放置合法性由后续布局算法维护、检查。

## 2. 读懂一个 row 的参数

公开接口位于 [odb/db.h](../../odb/include/odb/db.h)，实现位于 [dbRow.cpp](../../odb/src/db/dbRow.cpp)。创建接口如下：

```cpp
static dbRow* create(dbBlock* block,
                     const char* name,
                     dbSite* site,
                     int origin_x,
                     int origin_y,
                     dbOrientType orient,
                     dbRowDir direction,
                     int num_sites,
                     int spacing);
```

| 参数 | 含义 | 查询接口 |
| --- | --- | --- |
| `block` | 所属设计块 | `getBlock()` |
| `name` | 行名，如 `ROW_0` | `getName()` / `getConstName()` |
| `site` | 该行引用的 site 定义 | `getSite()` |
| `origin_x, origin_y` | 行的起始坐标 | `getOrigin()`，返回 `odb::Point` |
| `orient` | site 的朝向 | `getOrient()` |
| `direction` | site 沿 X 还是 Y 方向重复 | `getDirection()` |
| `num_sites` | 重复的 site 数量 | `getSiteCount()` |
| `spacing` | 相邻 site 原点之间的距离 | `getSpacing()` |

坐标、尺寸和 spacing 都使用 **DBU（数据库单位）**，不是直接使用微米。换算关系为：

```text
微米数 = DBU 数 / block->getDbUnitsPerMicron()
```

例如，当数据库每微米有 1000 DBU 时，500 DBU 就是 0.5 μm。不要把这个比例写死为某个工艺的值。

### direction 与 orient 是两个维度

- `HORIZONTAL`：沿 X 方向重复；`VERTICAL`：沿 Y 方向重复。
- `R0`：不旋转，在 DEF 中写作 `N`。
- `MX`：关于 X 轴镜像，在 DEF 中写作 `FS`。

一个 `HORIZONTAL + MX` 的 row 仍然是水平行，只是 site 的朝向发生镜像。当前 IFP 的普通建行路径生成水平行，并让相邻行在 `R0` / `MX` 间交替；这种安排通常用于配合标准单元库的电源轨布局。具体合法朝向还要结合库定义，不能仅凭 row 朝向判断单元一定能放进去。

OpenDB 能表示竖直行，并不代表每个下游算法都支持它。例如 DPL 的 `Opendp::createArchitecture()` 会跳过非水平行。

## 3. 行宽怎么计算？

对于水平行，设原点为 `(x, y)`，site 数量为 `n > 0`，间距为 `s`，朝向变换后的单个 site 尺寸为 `dx × dy`：

```text
第 i 个 site 的位置： (x + i × s, y)，i = 0 ... n-1
行的包围盒：          (x, y) → (x + (n-1) × s + dx, y + dy)
行宽：                (n-1) × s + dx
```

**`spacing` 是原点间距，不是两个 site 边缘之间的空隙。** 只有当 `s == dx` 时，行宽才简化为 `n × dx`。

例如，一个未旋转 site 宽 500 DBU、高 2000 DBU，row 从 `(1000, 4000)` 开始，包含 4 个 site，spacing 为 500 DBU：

```text
y=6000  ┌──────┬──────┬──────┬──────┐
        │  0   │  1   │  2   │  3   │
y=4000  └──────┴──────┴──────┴──────┘
x=      1000   1500   2000   2500   3000
```

包围盒是 `(1000, 4000) → (3000, 6000)`。如果 spacing 改成 600，行宽就变成 `3 × 600 + 500 = 2300` DBU；包围盒也会包含 site 之间的间隙，不能把整个包围盒都当作连续可放置空间。

`getBBox()` 还有几个值得知道的实现细节：

- 竖直行的高度为 `(n-1) × s + dy`，宽度为 `dx`。
- 它先用 `dbTransform` 得到朝向变换后的 site 尺寸；旋转 90° 时需要考虑宽高交换。
- 它取变换后的宽高，再从 row 的 origin 构造包围盒。因此 `MX` 不会让包围盒向 origin 下方延伸。
- 当 site 数量为 0 时，当前实现返回 `(0, 0, 0, 0)`。

## 4. 与 DEF 的 ROW 语句对照

上面的水平行若采用 `R0`，且 DEF 与数据库使用相同单位，写出结果为：

```def
ROW ROW_demo core_site 1000 4000 N DO 4 BY 1 STEP 500 0 ;
```

| DEF 片段 | 对应含义 |
| --- | --- |
| `ROW_demo` | row 名称 |
| `core_site` | site 名称，需能在已加载的库中找到 |
| `1000 4000` | 起始坐标 |
| `N` | `R0` 朝向 |
| `DO 4 BY 1` | X 方向 4 个位置，Y 方向 1 个位置 |
| `STEP 500 0` | X 方向步长 500，Y 方向步长 0 |

对于竖直行，OpenDB 写成 `DO 1 BY n STEP 0 spacing`。一个 `dbRow` 保存的是一维重复信息，没有独立的 X、Y 两组 count。

读取路径见 [definRow.cpp](../../odb/src/defin/definRow.cpp)：查找 site，将 DEF 距离转换成 DBU，再调用 `dbRow::create()`。写出路径见 [defout_impl.cpp](../../odb/src/defout/defout_impl.cpp) 的 `writeRows()`：遍历 `block->getRows()`，把 DBU 转换为输出 DEF 单位。发现输出坐标不对时，应先检查内存中的 row 和它的创建路径。

## 5. IFP 如何创建和调整 row？

主要代码位于 [InitFloorplan.cc](../src/InitFloorplan.cc)。初次阅读建议从 `makeRows()` 开始，再进入普通路径 `makeUniformRows()`。

```text
makeRows()
  → 检查 die/core、gap 和实例尺寸
  → prepareSitesAndClearRows()：收集 site，删除旧 row
  → 将 core 左下角向上对齐到 base site 网格
  → makeUniformRows() 或 makeHybridRows()
  → updateVoltageDomain()：根据电压域调整行
  → odb::cutRows()：根据 placement blockage 裁剪行
  → reportAreas()
```

### 普通行：网格计数与交替朝向

在普通路径中，当前实现用 **base site 的宽度**计算每行 site 数量及 spacing，用每个待铺 site 自己的高度计算行数：

```text
每行 site 数 = 对齐后 core 宽度 / base site 宽度（整数除法）
该 site 的行数 = core 高度 / 该 site 高度（再应用 row parity 约束）
第 j 行 Y 坐标 = 对齐后 core.yMin + j × 该 site 高度
```

不足一格宽或一行高的余量不会形成完整位置。`-flip_sites` 会交换所选 site 的起始朝向。普通路径还要求待铺 site 的高度是 base site 高度的整数倍。

这一步可能为多个 site 分别生成行，几何位置可以重叠，表示不同 site 的放置选择。因此，把所有 row 的面积或 site 数直接相加，不一定能得到唯一的可用容量。

### 混合高度与裁剪

当 base site 有 `ROWPATTERN` 时，`makeHybridRows()` 按模式中的 site、朝向和高度逐行生成，还会为带 row pattern 的 site 创建相应的行。一个 row 仍然只引用一个 site；模式信息保存在 site 中。

之后，电压域调整或 blockage 裁剪可能删除原 row，并创建较短的片段。因此，同一 Y 坐标可以对应多个 row，最终 row 数量也未必等于简单计算出的水平层数。

`dbRow::create()` 本身只负责保存信息和触发创建回调，不会自动完成 core 对齐、障碍物裁剪或单元合法性检查。手动创建一个 row 不等于完成了 IFP 建行流程。命令层面的参数说明可继续阅读 [make_rows 笔记](make_rows.md)。

## 6. 上手查看：Tcl 与 C++

### Tcl：查看当前设计的前 5 个 row

先在 OpenROAD 中加载包含 row 的设计，例如读取 floorplan DEF 或执行建行命令，再运行下面的只读代码：

```tcl
set block [ord::get_db_block]
set rows [$block getRows]
puts "row count = [llength $rows]"
puts "DBU per micron = [$block getDbUnitsPerMicron]"

foreach row [lrange $rows 0 4] {
  set site [$row getSite]
  puts "row=[$row getName] site=[$site getName]"
  puts "  origin(DBU)=[$row getOrigin]"
  puts "  direction=[$row getDirection] orient=[$row getOrient]"
  puts "  count=[$row getSiteCount] spacing(DBU)=[$row getSpacing]"
}
```

如果输出的 row 数量为 0，先确认当前设计是否已经读入或生成行；只加载 LEF 的 site 定义并不会生成 row。

### C++：遍历与获取包围盒

```cpp
#include <iostream>

#include "odb/db.h"

void printRows(odb::dbBlock* block)
{
  for (odb::dbRow* row : block->getRows()) {
    const odb::Rect bbox = row->getBBox();
    std::cout << row->getName() << " site=" << row->getSite()->getName()
              << " count=" << row->getSiteCount()
              << " bbox(DBU)=(" << bbox.xMin() << ", " << bbox.yMin()
              << ")-(" << bbox.xMax() << ", " << bbox.yMax() << ")\n";
  }
}
```

### 生命周期与内部存储

`block->getRows()` 返回 `dbSet<dbRow>`。内部对象 `_dbRow` 存在 block 的 `row_tbl_` 中，通过 `lib_` 和 `site_` 两个数据库 ID 引用 site，而不是为每个格子复制一份 site 对象。持久化字段见 [dbRow.h](../../odb/src/db/dbRow.h)。

公开接口没有修改 origin、spacing 等字段的 setter。需要替换行时，通常重新创建；销毁必须使用 `dbRow::destroy()`，不要直接 `delete`。销毁会触发回调、清理属性并从表中移除对象，旧指针不能继续使用。

遍历中删除时，使用返回下一位置的迭代器重载。下面是**删除全部 row**的模式，仅用于理解生命周期，不要作为查看脚本运行：

```cpp
auto rows = block->getRows();
for (auto it = rows.begin(); it != rows.end();) {
  it = odb::dbRow::destroy(it);
}
```

`dbRow::getRow(block, oid)` 按数据库 ID 取回对象，不是按名字查找。当前 `create()` 也不检查名称重复；行名不能被当作数据库 ID 使用。

## 7. 排查问题时先看什么？

| 现象 | 优先检查 |
| --- | --- |
| 坐标比预期大很多 | DBU 与微米是否混用，DEF 输出单位是否一致 |
| 行起点与输入 core 左下角不同 | `makeRows()` 的网格对齐逻辑 |
| 行宽与 `count × site width` 不同 | spacing、朝向，以及 `(count-1) × spacing + dx` 公式 |
| row 数比预期多 | 多 site 重叠行、hybrid 行、裁剪后的片段 |
| row 数比预期少 | 整数除法余量、row parity、blockage、电压域调整 |
| row 存在但单元无法合法放置 | 单元与 site 的匹配、尺寸、朝向、占用和下游算法约束 |
| 手动保存的 row 指针突然失效 | 是否重新执行建行或进行了删行、裁剪 |

若要跟踪下游消费方式，可看 [dbToOpendp.cpp](../../dpl/src/dbToOpendp.cpp) 的 `createArchitecture()`：它读取 row 的原点、site 数量、spacing 等，构造详细布局使用的内部行结构。

## 8. 自测与后续阅读

1. 一行有 100 个 site，是否一定能放 100 个单元？
2. 水平行包含 5 个 site，site 宽 400 DBU，spacing 为 500 DBU，朝向为 `R0`，行宽是多少？
3. 把朝向从 `R0` 改成 `MX`，会变成竖直行吗？
4. 重新执行 `make_rows` 后，还能继续使用之前缓存的 row 指针吗？

参考答案：

1. 不能。单元可能占多个 site 宽度，还要考虑已有占用和其他约束。
2. `4 × 500 + 400 = 2400` DBU。
3. 不会，重复方向由 `direction` 决定。
4. 不能。该路径会删除旧 row，需要重新获取对象。

建议接着阅读 [dbSite 笔记](dbSite.md) 和 [dbTrackGrid 笔记](dbTrackGrid.md)，区分放置格子、放置行和布线轨道。仓库中的 [row_settings.tcl](../../odb/test/row_settings.tcl) 展示了 row 的创建和 getter 检查；[make_rows.tcl](../test/make_rows.tcl) 展示了从已有设计建行并写出 DEF 的流程。
