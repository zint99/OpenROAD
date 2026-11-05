# 使用流程与检查

[返回入口](README.md) · [实现边界与源码导航](implementation.md)

## 1. 输入文件如何变成 ODB 对象

| 输入 | 主要内容 | 读入后的对象 |
| --- | --- | --- |
| `.3dbv` | 芯片类型、尺寸、region 和外部文件引用 | `dbChip`、`dbBlock`、`dbChipRegion` |
| region 引用的 `.bmap` | bump 单元、位置、网络和端口 | `dbInst`、`dbChipBump` 及相关网络、端口 |
| `.3dbx` | 顶层设计、芯片实例、堆叠和键合关系 | `HIER` 顶层、`dbChipInst`、`dbChipConn` |
| 顶层 Verilog | 芯片实例之间的端口连接 | `dbChipNet` |
| LEF / DEF | 工艺、单元几何和芯片内部物理设计 | `dbTech`、`dbLib`、`dbBlock` 内部对象 |

`.3dbx` 可通过 `Header.include` 引用 `.3dbv` 或其他 `.3dbx`，导入器递归读取。通常从系统顶层 `.3dbx` 开始，不需要先把其 include 的文件再手动读一遍。

完整的 `read_3dbx` 命令会依次完成：解析输入、建立层次与连接、构建展开视图、发送读入完成回调、运行检查。仅调用底层 C++ `ThreeDBlox::readDbx()` 不包含这些外围步骤。

## 2. 跑通仓库自带示例

下面在仓库根目录执行，假设可执行文件位于 `build/bin/openroad`：

```sh
build/bin/openroad -no_init
```

在 OpenROAD 的 Tcl 提示符中执行：

```tcl
read_3dbx src/odb/test/data/example.3dbx
report_3dic_summary

set db [ord::get_db]
set top [$db getChip]
puts "Top: [$top getName]"
foreach inst [$top getChipInsts] {
  set master [$inst getMasterChip]
  puts "[$inst getName] -> [$master getName]"
}

# 保存数据库快照，文件路径可以自行替换。
write_db /tmp/odb_3dic_example.odb
```

示例顶层名为 `TopDesign`，其中 `soc_inst` 和 `soc_inst_duplicate` 均引用 `SoC`。它展示同一芯片的重复放置与 Z 方向翻转；它不是完整产品设计的签核样例。

在本文基线版本上，示例输出为 2 个 chiplet 实例、0 条顶层 chip-net，bump pads 为 4（0 connected）。还会出现 `STA-3004` / `STA-3000` 警告：当前 STA 不支持同一 chiplet master 重复放置，因此不创建该设计的时序网络，但 ODB 结构数据仍可使用。这些现象不表示上述 Tcl 示例执行失败，也不能据此认为示例已经完成跨芯片信号连接。

`report_3dic_summary` 适合快速确认数据是否读入，但其统计不能替代实际连接检查。输入示例还包含 LEF、Liberty、DEF 等依赖，使用时应保留仓库中的相对目录结构。

## 3. 修改后再检查：不要漏掉重建

下面接着上面的会话运行。将第二颗 SoC 放回示例位置，演示 API 使用；坐标值是 DBU，而非微米：

```tcl
set inst [$top findChipInst soc_inst_duplicate]
set dbu [$db getDbuPerMicron]
set p [odb::Point3D]
$p set [expr {100 * $dbu}] [expr {200 * $dbu}] [expr {300 * $dbu}]
$inst setLoc $p

$db constructUnfoldedModel
check_3dblox
```

当前 `check_3dblox` **不会自行重建展开模型**。修改后直接调用检查，可能使用旧的展开数据。直接通过 C++ API 建模时也需要遵守这个顺序。

检查结果放在顶层 `dbChip` 的 `3DBlox` marker 分类下。每次检查会替换该分类下的旧结果；可结合日志和 GUI 中的 marker 定位问题。

## 4. 当前检查究竟检查什么

以下内容以 `Checker::check()` 实际调用的函数为准：

| 检查项 / marker 分类 | 当前实际含义 |
| --- | --- |
| `Logical Connectivity` | 对有效键合连接上 XY 坐标完全一致的上下 bump，比较其展开网络归属是否一致 |
| `Floating chips` | 基于有效连接检测分离的芯片集合；虚拟连接可接到外部，未指定外部连接时以最大集合为基准 |
| `Overlapping chips` | 检测芯片三维体积重叠 |
| `Unused internal_ext` | 检测没有被连接引用的 `INTERNAL_EXT` 区域 |
| `Connection regions` | 检查连接面的方向、XY 重叠和间距；内部可外接区域还涉及 Z 重叠 |
| `Bump Alignment` | 检查 bump 的全局中心是否位于所属 region 的 XY 范围内 |
| `Alignment Markers` | 根据注册规则检查键合芯片上的对准标记，包括配对距离和可选的相对方向 |

特别注意：

- `Bump Alignment` 不是上下 bump 的完整配对检查。
- 逻辑检查会跳过找不到同 XY 对端的 bump；两端都没有网络时，也不会因网络对象不同而报错。因此检查无报错并不保证所有信号都已接通。
- `checkNetConnectivity()` 当前为空，且没有被 `check()` 调用。
- `dbChipPath` 可以保存路径断言，但当前检查器未消费这些断言。

对准标记规则使用 `add_3dblox_alignment_marker_rule` 注册，`-master_a` 和 `-master_b` 指定单元，`-tolerance` 使用微米。完整参数见 [ODB 命令说明](../../README.md#add-3dblox-alignment-marker-rule)。

## 5. 常见问题按这个顺序排查

| 现象 | 优先检查 |
| --- | --- |
| 找不到芯片定义或 bump 单元 | include 和 LEF 路径是否正确，实例 reference 是否与定义名一致 |
| 芯片位置或大小差很多 | 是否把微米直接传给 DBU API，是否漏掉父级变换 |
| 正面明明存在，却提示连接方向错误 | 芯片是否翻转；上方芯片的连接面应朝下，下方芯片的连接面应朝上 |
| 看起来接触，仍提示区域连接错误 | XY 是否重叠、表面间距是否与 `thickness` 一致 |
| 有物理连接但没有预期网络 | 顶层 Verilog 是否存在且成功链接；实例名、端口名和 bump 的 `dbBTerm` 是否匹配 |
| 修改后检查结果不对 | 是否重建展开模型，并重新获取展开对象 |
| 同一 chiplet 的多个实例想读入不同 DEF | 它们共享 master 的 `dbBlock`；当前导入器限制每个芯片定义只指定一次 DEF |

## 6. 保存与导出

- `write_db` / `read_db`：保存、恢复 ODB 数据库。展开模型不作为持久化数据写入；数据库读取路径会在多芯片条件下重建，直接 API 工作流也可显式重建。
- `write_3dbx <文件>`：导出装配描述，并生成关联的 `.3dbv`、Verilog 等配套文件。
- `write_3dbv <文件>`：导出相关芯片定义，并生成使用到的技术 / 库 LEF、bmap 等文件。

导出 3DBlox 时建议使用单独的输出目录，不能只保留一个 `.3dbx` 文件就当作完整交付。需要保留数据库状态时优先使用 `.odb`，不要假设文本导出与数据库快照完全等价。

另一个容易混淆的入口是 `read_3dblox_bmap`：它面向当前顶层 block 上已有网络、端口的 bump 导入，并生成相关 BPin。它与 `.3dbv` 的 region bmap 导入路径不同，不应当作给 `HIER` 装配顶层添加 region / bump 的通用命令。
