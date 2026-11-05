# 实现边界与源码导航

[返回入口](README.md) · [使用流程与检查](usage.md)

## 1. 当前能力边界

| 能力 | 当前实现 |
| --- | --- |
| 多芯片定义、重复实例化、层次组合和多工艺引用 | 已有 ODB 数据模型 |
| 三维放置、Z 翻转、全局展开视图 | 已实现；编辑后需要显式重建展开视图 |
| Region、bump、物理键合与跨芯片网络 | 已实现，分别保存几何关系与逻辑关系 |
| `.3dbv` / `.3dbx` / bmap 读写和 DEF 合入 | 已有实现；支持范围应以解析器实际使用的字段为准 |
| 路径断言 `dbChipPath` | 已解析并存储，当前 Checker 尚未检查 |
| 通用网络连通检查 | `checkNetConnectivity()` 仍是空函数 |
| 芯片级 RC 节点、电阻段及数据库读写 | 已实现，并有单元测试 |

### RC 数据模型与 RCX 的区别

ODB 提供 `dbChipNet` 上的 RC 容器，RCX 负责填入提取结果。当前 `MultiChipExtractor` 的跨芯片部分遍历顶层 chip-net，对两个 bump 建立两个电容节点和一个电阻段，电阻来自装配规则，暂忽略 bump 电容贡献。

这条提取路径对超过两个 bump 的 chip-net 报错，并明确拒绝 RDL 类型芯片。ODB 中存在 `RDL` 类型，并不意味着这条 RCX 路径已经支持 RDL。当前也不能把其顶层双端点处理理解成任意嵌套层次网络的通用提取。

RCX 的 `MultiChipSpefWriter` 已读取芯片级 RC 数据用于输出；“跨芯片寄生参数尚未实现”的旧说法已不适用。

### 解析成功不等于所有功能都已执行

当前 `buildChipNetsFromVerilog()` 从设计顶层网络出发，按直接子实例名、master block 的端口名和端口对应的 bump 建立 chip-net。它不是任意深度 RTL 层次的通用自动展开器。文件缺失、链接失败或端口无法匹配时，可能得不到预期网络，应检查实际数据库内容。

同样，输入文件中出现某个外部文件字段，不代表对应分析已经执行。例如，不能仅凭 `.3dbx` 中声明了 SDC 路径就判断约束已载入并完成跨芯片 STA。

ODB 支持重复实例化芯片，但当前 dbSta 建立 3DIC 时序网络时会拒绝重复放置的 master，发出 `STA-3004`。这是时序网络的限制，不是 ODB 层次模型的限制，判断逻辑见 [dbNetwork.cc](../../../dbSta/src/dbNetwork.cc)。

## 2. 阅读源码的最短路径

所有链接均相对于本文位置，点击即可定位到实现文件。

| 想了解什么 | 首选入口 | 阅读重点 |
| --- | --- | --- |
| 对外有哪些对象和接口 | [db.h](../../include/odb/db.h) | `dbChip*` 与 `dbUnfoldedChip*` 类 |
| Tcl 命令如何进入 C++ | [OpenRoad.tcl](../../../OpenRoad.tcl)、[OpenRoad.cc](../../../OpenRoad.cc) | `read_3dbx`、`read3Dbx()`、`check3DBlox()` |
| 文件如何转成数据库对象 | [3dblox.cpp](../../src/3dblox/3dblox.cpp) | `readDbx()`、`createChiplet()`、`buildChipNetsFromVerilog()` |
| 芯片实例如何生成 region / bump 实例 | [dbChipInst.cpp](../../src/db/dbChipInst.cpp) | `dbChipInst::create()` |
| 如何计算全局层次视图 | [dbUnfoldedBuilder.cpp](../../src/db/dbUnfoldedBuilder.cpp) | `build()`、递归变换、路径解析 |
| bump 的全局位置如何计算 | [dbUnfoldedChipBumpInst.cpp](../../src/db/dbUnfoldedChipBumpInst.cpp) | XY 使用底层 bump 单元包围盒中心，Z 来自 region 表面 |
| 检查器到底验证了什么 | [checker.cpp](../../src/3dblox/checker.cpp) | 先看 `Checker::check()`，再看对应检查函数 |
| 数据库加载时如何恢复展开视图 | [dbDatabase.cpp](../../src/db/dbDatabase.cpp) | 读取路径与 `constructUnfoldedModel()` |
| RC 的数据结构与生命周期 | [dbChipCapNode.cpp](../../src/db/dbChipCapNode.cpp)、[dbChipRSeg.cpp](../../src/db/dbChipRSeg.cpp) | 节点归属、bump 引用和端点约束 |
| 哪个模块填入和导出跨芯片 RC | [multiChipExtractor.cpp](../../../rcx/src/multiChipExtractor.cpp)、[multiChipSpefWriter.cpp](../../../rcx/src/multiChipSpefWriter.cpp) | 双端点键合模型和 SPEF 输出 |

若要扩展数据库字段，还应查看 [chip schema](../../src/codeGenerator/schema/chip)。部分数据库代码由生成器维护，修改时需遵守生成区和用户代码区约定。

## 3. 从测试中找可复用的例子

| 测试 | 适合学习什么 |
| --- | --- |
| [TestChips.cpp](../../test/cpp/TestChips.cpp) | 层次、region / bump、连接、RC 创建销毁及序列化 |
| [Test3DBloxParser.cpp](../../test/cpp/Test3DBloxParser.cpp) | 文件解析与输入处理 |
| [Test3DBloxChecker.cpp](../../test/cpp/Test3DBloxChecker.cpp) | 空间关系与区域检查 |
| [Test3DBloxCheckerLogicalConn.cpp](../../test/cpp/Test3DBloxCheckerLogicalConn.cpp) | 物理对齐与逻辑网络一致性 |
| [Test3DBloxCheckerBumps.cpp](../../test/cpp/Test3DBloxCheckerBumps.cpp) | bump 区域边界检查 |
| [Test3DBloxCheckerAlignmentMarkers.cpp](../../test/cpp/Test3DBloxCheckerAlignmentMarkers.cpp) | 对准标记规则 |
| [Test3DBloxVerilogWriter.cpp](../../test/cpp/Test3DBloxVerilogWriter.cpp) | 系统级 Verilog 输出 |
| [check_3dblox.tcl](../../test/check_3dblox.tcl) | 修改放置、重建展开模型、再次检查的完整顺序 |

调试时先判断问题属于“原始数据”“层次路径”“坐标变换”还是“检查规则”，再进入对应实现。发现输出有误，应追到对象创建或关联的位置，而不只是修改导出文本。
