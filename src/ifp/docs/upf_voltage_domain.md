# IFP 中的 UPF 与电压域：从电源意图到放置行

> 面向新员工，建议先了解 [dbRow](dbRow.md) 和 [make_rows](make_rows.md)。本文讲解当前仓库的实现与阅读方法，不是 UPF 标准或完整低功耗设计流程的说明。

## 1. 先理解 IFP 在做什么

多电源域设计需要回答两类问题：哪些实例属于同一电源域、跨域信号如何处理；这些域在芯片上占据哪里、域内和域外可以在哪里放置单元。

**IFP 在建行时先调用 UPF 模块落实电源意图，再根据域的物理区域切分放置行，并在域周围留出间隔。** 前一步可能改变实例和网络连接，后一步主要改变 `dbRow`。

例如，core 中央有一个独立电源域。IFP 可以把穿过它的一整行变成“左侧普通行、域内行、右侧普通行”，中间留出空隙。域的上下边缘附近还会移除中央的行片段。这里创建的是放置空间，电源环、电源条带和供电连接属于其他流程；行切分本身不会生成 PDN。

## 2. 四种对象，以及两条入口

| 对象 | 在这里承担的职责 | 关键访问方式 |
| --- | --- | --- |
| `dbPowerDomain` | 保存域的 elements、area、voltage 等电源意图 | `block->getPowerDomains()` |
| `dbGroup` | 将实例组织为一组，并关联物理 region | `group->getInsts()`、`getRegion()`、`getType()` |
| `dbRegion` | 保存域的物理边界，可含多个 boundary box | `region->getBoundaries()` |
| `dbRow` | 保存实际放置行的坐标、site、朝向和格数 | `block->getRows()` |

### 入口 A：UPF 电源意图

```text
read_upf / create_power_domain / set_isolation / set_level_shifter ...
  → dbPowerDomain 和相关策略对象
set_domain_area
  → dbPowerDomain 中的 area
IFP 调用 eval_upf()
  → 非顶层域对应的 dbRegion + POWER_DOMAIN 类型 dbGroup
  → 实例归组，以及按策略进行的跨域连接处理
```

当前 `read_upf` 在 Tcl 层通过 `source` 执行文件；它不是 IFP 的行切分函数。`set_domain_area` 将输入微米转换成 DBU，然后设置 `dbPowerDomain` 的 area。物理 group/region 的建立发生在后面的 `eval_upf()` 中。

### 入口 B：直接定义 voltage domain

```tcl
# 已加载并 link 设计后，坐标单位为微米。
create_voltage_domain TEMP_ANALOG -area {34 34 66 66}
```

该命令位于 OpenDB 的 Tcl 文件 [odb.tcl](../../odb/src/swig/tcl/odb.tcl)，直接创建 region、boundary box 和 `VOLTAGE_DOMAIN` 类型 group。它不会创建 `dbPowerDomain`，也不会根据层级元素给实例自动归组或定义 isolation/level shifter 策略。

两条入口最终汇入 IFP 的同一个条件：

```cpp
group->getType() == dbGroupType::VOLTAGE_DOMAIN
    || group->getType() == dbGroupType::POWER_DOMAIN
```

因此，**没有 UPF 文件也可以触发行切分**。反过来，仅有 UPF 逻辑域定义，还不足以提供有效的物理区域。不要为了同一个域同时用两条入口创建同名 region/group，否则可能发生名称冲突。

## 3. 在建行流程中的调用顺序

矩形 core 的主流程见 [InitFloorplan.cc](../src/InitFloorplan.cc) 中的 `makeRows()`：

```text
检查 gap、die/core 和实例尺寸
  → prepareSitesAndClearRows()：收集 site 并删除旧行
  → eval_upf(network_, logger_, block_)
  → 将 core 左下角对齐到 base site 网格
  → makeUniformRows() 或 makeHybridRows()
  → updateVoltageDomain(clx, cly, cux, cuy, gap)
  → odb::cutRows()：处理 placement blockage
  → reportAreas()
```

UPF 求值、铺行和域切分位于 core 左下角非负的条件分支中。`initialize_floorplan` 的矩形路径和 `make_rows` 都会进入这套建行逻辑。

多边形路径 `makePolygonRowsScanline()` 同样先调用 `eval_upf()`，生成多边形行后调用 `updateVoltageDomain()`，最后处理 blockage。需要注意，它传入域切分函数的是 core 的包围盒边界，相关限制见第 8 节。

这个顺序有两个实际影响：

- 建行前会删除旧 row，缓存的旧 row 指针不能继续使用。
- site 收集发生在 UPF 求值之前。若调查 UPF 新增单元与 site 的兼容性，要同时查看这个顺序，不能假定新增实例已经参与了此前的 site 扫描。

## 4. eval_upf() 到底做了什么？

实现位于 [upf.cpp](../../upf/src/upf.cpp)。对 IFP 开发者，优先读 `eval_upf()`、`associate_groups()` 和 `add_insts_to_group()`。

### 4.1 建立域与物理对象的关系

`associate_groups()` 根据 power domain 的 elements 建立路径映射。包含 `.` 的域被标为顶层域；它在这里不会创建自己的 region/group。非顶层域则创建：

1. 同名 `dbRegion`，类型设为 `EXCLUSIVE`。
2. 根据 `dbPowerDomain::getArea()` 创建 region 的 boundary box。
3. 同名 `dbGroup`，类型设为 `POWER_DOMAIN`，并保存到 power domain 中。

若多个域声明同一路径，报 `UPF 19`；如果定义了 power domain 却没有顶层域，报 `UPF 29`。没有设置 area 时会发出 `UPF 21` 警告，但此处仍继续建立 group，因此不能把这个警告理解为“工具已经推导出区域”。

### 4.2 按实例层级归组

`build_domain_hierarchy()` 和 `match_module_to_domain()` 沿模块层级建立对应关系；当前匹配逻辑选取字符串最长前缀，未匹配到时回退至顶层域。`add_insts_to_group()` 再将具有相应域父级关系的实例加入该域 group。

这个归属来自逻辑层级，不是根据实例当前坐标判断。区域内恰好有一个实例，不代表它自动属于该电源域。

### 4.3 按策略处理连接

`eval_upf()` 开始时会为 UPF logic port 尝试创建 net 和顶层端口。若没有 power domain，随后直接返回成功；因此“没有域”也不严格等于函数完全没有副作用。

有域时，它遍历非顶层域实例的连接。当前主要分支是：目标域为空或两域电压相等时，调用 `isolate_connection()`；电压不同时，查找并验证 level shifter 策略，再尝试插入转换单元。实际是否插入，还取决于策略、端口方向、单元映射等检查。

所以运行 floorplan 后发现实例或网络数变化，应先检查 UPF 求值。不要把“电压不同”理解为一定插入 level shifter，也不要把这里的条件当作完整的 UPF 标准规则。

## 5. updateVoltageDomain() 如何切行？

该函数不读取实际电压值，也不选择 isolation cell。它读取 group、region 和 row，执行几何处理。

### 5.1 将 region 转成处理矩形

函数遍历两种域类型的 group，取得 region 的所有 boundary，分别取最小的 X/Y 下界和最大的 X/Y 上界，得到一个**外包矩形**。

若 region 包含两个分离的矩形，这里会把中间空白也包含进来；它没有按各个 boundary 的精确并集切行。这个矩形随后只在局部变量中对齐，不会写回 region 或 power domain 的 area。

### 5.2 选择对齐单位和 gap

先收集当前所有非 `PAD` site 的 row。如果没有可处理行，就跳过这个域。再从这些行的 site 中分别找出最小宽度 `min_site_dx` 和最小高度 `min_site_dy`。

默认间隔为：

```text
gap = 6 × min_site_dy
```

用户提供的 `-gap` 在 Tcl 中从微米转换为 DBU。未提供时用 `INT32_MIN` 表示“使用默认值”；显式值必须为正，否则 `checkGap()` 报 `IFP 36`。

虽然局部变量叫 `power_domain_y_space`，**它也用于域左右两侧的留白**，并非只影响 Y 方向；它同样不是所有相邻 row 之间的固定间距。

域边界向内对齐，网格偏移量为 0：

```text
L = ceil(原左界 / min_site_dx) × min_site_dx
R = floor(原右界 / min_site_dx) × min_site_dx
B = ceil(原下界 / min_site_dy) × min_site_dy
T = floor(原上界 / min_site_dy) × min_site_dy
```

对应 [util.cpp](../../odb/src/db/util.cpp) 的 `makeSiteLoc()`：布尔参数为 `false` 时向上取整，为 `true` 时向下取整。阅读时直接看实现比根据参数名猜方向更可靠。

### 5.3 逐行判断，最多创建三段

设原行的 Y 范围为 `[y0, y1]`，间隔为 `g`。满足下面任意条件的行保持原状：

```text
y1 + g <= B
y0 >= T + g
```

其余行进入切分路径：保存名称、site 和朝向，销毁原 row，然后按以下规则创建新行。

| 片段 | 起点及长度计算 | 创建条件与名称 |
| --- | --- | --- |
| 左侧普通行 | 从 `core_lx` 开始，site 数为 `floor((L-g-core_lx)/site_dx)` | `L-g > core_lx+site_dx`；原名加 `_1` |
| 右侧普通行 | 起点为向右对齐到该 site 网格的 `R+g`，记为 `r`；site 数为 `floor((core_ux-r)/site_dx)` | `r+site_dx < core_ux`；原名加 `_2` |
| 域内行 | 从 `L` 开始，site 数为 `floor((R-L)/site_dx)` | 原行完全落入域的 Y 范围，即 `y0 >= B && y1 <= T`；原名加 `_域名` |

新行沿用原来的 site、Y 起点和朝向，方向统一设为 `HORIZONTAL`，spacing 设为该 site 的宽度。左右片段条件是严格不等式，恰好只余一个 site 宽度时不会创建；不足 10 个 site 宽的左右空间可能分别触发 `IFP 26` / `IFP 27`，日志提示 tapcell 插入可能受影响。

```text
域上方留白带：  左侧普通行 |       中央无行       | 右侧普通行
域内部：        左侧普通行 | gap | 域内行 | gap | 右侧普通行
域下方留白带：  左侧普通行 |       中央无行       | 右侧普通行
```

row 名称中的域名只是命名结果；该函数没有把 row 加入 group，实例归属仍由 group 中的实例成员表示。

## 6. 手算一个例子

假设所有 row 使用宽 1 μm、高 2 μm 的 site，core 为 `(0,0) → (100,80)`，域原始区域为 `(30.2,20.3) → (69.8,59.7)`，显式设置 `-gap 4`。为便于阅读，本节用微米计算；源码内部使用 DBU。

向内对齐后，域处理矩形为 `(31,22) → (69,58)`。左侧普通行终点最多到 `31-4=27`，右侧普通行从 `69+4=73` 开始。

| 原行 Y 范围 | 处理结果 |
| --- | --- |
| `[16,18]` | `18+4 <= 22`，原行不变 |
| `[18,20]` | 位于留白带，生成 X 范围 `[0,27]`、`[73,100]` 两段 |
| `[22,24]` | 完全位于域内，另生成 X 范围 `[31,69]` 的域内行 |
| `[58,60]` | 不完全位于域内，只保留左右两段 |
| `[62,64]` | `62 >= 58+4`，原行不变 |

域内部的那一层共生成 3 个 row 对象，site 数分别为 27、38、27。若不指定 gap，这个例子的默认值会是 `6 × 2 = 12 μm`，留白明显更大。

## 7. 如何上手观察与读测试

UPF 路径的脚本顺序通常如下。这里是结构示意，文件、域名和 site 名需要来自实际设计；UPF 应定义顶层域、子域及所需策略。

```tcl
# 先加载 Liberty、LEF、Verilog，并执行 link_design。
read_upf -file design.upf
set_domain_area PD_CHILD -area {30 20 70 60}
initialize_floorplan -die_area {0 0 120 100} \
  -core_area {10 10 110 90} -site core_site -gap 4
```

在建行后用下面的只读代码确认物理域已经形成；boundary 坐标输出为 DBU：

```tcl
set block [ord::get_db_block]
foreach group [$block getGroups] {
  set type [$group getType]
  if { $type ni {POWER_DOMAIN VOLTAGE_DOMAIN} } {
    continue
  }
  puts "domain=[$group getName] type=$type insts=[llength [$group getInsts]]"
  set region [$group getRegion]
  if { $region != "NULL" } {
    foreach box [$region getBoundaries] {
      puts "  boundary(DBU)=[$box xMin] [$box yMin] [$box xMax] [$box yMax]"
    }
  }
}
puts "rows=[llength [$block getRows]]"
```

观察行坐标可接着使用 [dbRow 笔记中的查看示例](dbRow.md)。region 的坐标是原始物理边界，未必等于 IFP 内部向内对齐后的域行边界。

建议按下面的顺序阅读已有测试：

| 测试 | 学习重点 |
| --- | --- |
| [init_floorplan8.tcl](../test/init_floorplan8.tcl) | 直接创建 voltage domain，再建行并比较 DEF |
| [init_floorplan9.tcl](../test/init_floorplan9.tcl) | 非整齐域坐标下的行边界 |
| [init_floorplan_gap.tcl](../test/init_floorplan_gap.tcl) | 非正 gap 报错、显式 gap、多高度 site |
| [init_floorplan_dbl_row.tcl](../test/init_floorplan_dbl_row.tcl) | 电压域与双高度行 |
| [upf_test.tcl](../test/upf_test.tcl) | UPF 域、isolation、建行前后的实例归组与连接 |
| [upf_shifter_test.tcl](../test/upf_shifter_test.tcl) | 域电压、level shifter 策略与连接变化 |

配套的 [mpd_top.upf](../test/upf/mpd_top.upf) 展示 isolation 策略，[mpd_shifter.upf](../test/upf/mpd_shifter.upf) 展示转换方向、阈值和单元映射。后者使用测试用的普通逻辑单元来验证连接机制，不应直接照搬为真实工艺的电平转换单元选择。

## 8. 当前实现的边界与排查提示

以下结论来自当前源码的控制流，帮助定位问题；不代表已经通过新增运行实验验证了所有组合场景。

**不要假定 UPF 求值可以无副作用地重复执行。** `eval_upf()` 有清理和重建相关的 TODO，但当前会直接创建 region/group。再次调用可能因重名返回失败。IFP 的两个调用点没有检查其布尔返回值，因此应检查 UPF 日志与实际数据库状态，不能仅以“建行命令继续执行”判断求值成功。

**物理区域需要有效边界。** `updateVoltageDomain()` 直接访问 group 的 region，以整数极值初始化边界累积变量；此处没有专门处理空 region、空 boundary、向内对齐后为空的区域，也没有先把域裁剪到 core 内。遇到缺少 area、过小区域或越界区域，应先核对输入和数据库对象。

**这段切分逻辑有全宽水平行的假设。** 是否切行只判断 Y 范围，左右片段使用全局 `core_lx/core_ux`，而不是原 row 的 X 范围。多个域按 group 遍历顺序依次处理，后一个域会看到前一个域新建的片段。因此不能把它理解为对任意多个重叠域进行严格几何相减的通用算法。

**多边形和已切短的行要特别检查结果。** 多边形路径传入的是 core 包围盒；结合上述左右端点计算，有重新生成超出原行片段范围的可能。后续 `cutRows()` 处理的是 placement blockage，不能据此保证恢复多边形边界。

**多高度 site 的对齐分两层。** 域边界按全局最小 site 宽高对齐，但每个新行的 count 用自身 site 宽度计算；域内行仍从统一的 `L` 开始。不要推断最小网格上的点一定满足所有 site 的对齐要求。

排查时依次查看：power domain 的 elements/area → group 类型和实例成员 → region boundary → 对齐后的局部边界和 gap → 新建 row 的坐标与 site 数。连接异常从 `eval_upf()` 往下追，几何异常从 `updateVoltageDomain()` 的输入和重建参数往下追。

## 9. 自测

1. `create_voltage_domain` 是否等同于读取 UPF？
2. site 最小高度为 1.4 μm，未指定 gap 时域两侧使用多少间隔？
3. 一行只与域边界部分相交，会生成域内行吗？
4. region 有两个分离的 boundary，这段 IFP 逻辑会分别切两个矩形吗？
5. 建行后出现新的隔离实例，应先查看哪段代码？

参考答案：

1. 不等同。它直接建立物理 region/group，不建立完整 UPF 意图和策略。
2. `6 × 1.4 = 8.4 μm`，同一个 gap 也用于上下方向的判定。
3. 不会；原行 Y 范围必须完整包含在对齐后的域 Y 范围内。
4. 不会；当前使用所有 boundary 的外包矩形。
5. `upf::eval_upf()` 及其 `isolate_connection()` 调用路径。
