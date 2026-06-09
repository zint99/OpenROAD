# `read_3dbx` 读取与 ODB 建库流程

本文以 `src/odb/test/read_3dbx.tcl` 为入口，梳理 OpenROAD 读取
`.3dbx` 文件的调用链，以及根据 `.3dbx` / `.3dbv` / `.bmap` 等 3DBlox
输入建立 OpenDB 对象的过程。

## 测试入口

`src/odb/test/read_3dbx.tcl` 的测试逻辑很短：

```tcl
source "helpers.tcl"

set db [ord::get_db]

read_3dbx "data/example.3dbx"
if { [$db getChip] == "NULL" } {
  puts "FAIL: Read 3dbx Failed"
  exit 1
}

puts "pass"
```

测试只验证一件事：执行 `read_3dbx data/example.3dbx` 后，
`dbDatabase::getChip()` 必须返回非空 chip。这说明 `.3dbx` 读取流程至少需要成功
创建并设置顶层 `dbChip`。

## 总体调用链

```text
read_3dbx "data/example.3dbx"
  -> src/OpenRoad.tcl: read_3dbx
     -> 参数个数检查、路径规范化、文件存在性/可读性检查
     -> ord::read_3dbx_cmd
       -> src/OpenRoad.i: read_3dbx_cmd
          -> OpenRoad::read3Dbx(filename)
            -> ThreeDBlox parser(logger_, db_, sta_)
            -> ThreeDBlox::readDbx(filename)
               -> DbxParser::parseFile(filename)
               -> readHeaderIncludes(Header.include)
               -> createDesignTopChiplet(Design)
               -> createChipInst(ChipletInst + Stack)
               -> buildChipNetsFromVerilog(Design.external.verilog_file)
               -> createConnection(Connection)
               -> calculateSize(top_chip)
               -> create dbChipPath(Path)
            -> dbDatabase::triggerPostRead3Dbx(db_->getChip())
            -> OpenRoad::check3DBlox()
```

其中 `ThreeDBlox::readDbx()` 是核心建库入口。`read_3dbx` 命令本身不直接操作
ODB，它只通过 Tcl/SWIG 桥接进入 C++。

## Tcl 与 C++ 入口

`src/OpenRoad.tcl` 中的 `read_3dbx`：

- 使用 `sta::parse_key_args` 和 `sta::check_argc_eq1` 要求正好一个参数。
- 使用 `file nativename` 规范化文件名。
- 检查文件是否存在、是否可读。
- 调用 `ord::read_3dbx_cmd $filename`。

`src/OpenRoad.i` 中的 SWIG 命令只做桥接：

```cpp
void
read_3dbx_cmd(const char *filename)
{
  OpenRoad *ord = getOpenRoad();
  ord->read3Dbx(filename);
}
```

`src/OpenRoad.cc` 中的 C++ 入口：

```cpp
void OpenRoad::read3Dbx(const std::string& filename)
{
  odb::ThreeDBlox parser(logger_, db_, sta_);
  parser.readDbx(filename);
  db_->triggerPostRead3Dbx(db_->getChip());
  check3DBlox();
}
```

这里可以看到读取后的两个后处理动作：

- `triggerPostRead3Dbx()`：通知数据库 observer，并构建 unfolded model。
- `check3DBlox()`：自动运行 3DBlox checker。

## `.3dbx` 文件在流程中的角色

`.3dbx` 是 3DIC 顶层装配文件。以 `src/odb/test/data/example.3dbx` 为例，它包含：

- `Header`：版本、单位、DBU 精度、include 文件。
- `Design`：顶层设计名，以及可选顶层 Verilog 文件。
- `ChipletInst`：实例名到 chiplet master 的引用关系。
- `Stack`：每个 chiplet 实例的 `(x, y, z)` 位置和 3D 朝向。
- `Connection`：不同 chiplet region 之间的垂直/物理连接。
- `Path`：可选路径断言，创建为 `dbChipPath`。

`.3dbx` 不完整描述一个 chiplet 的几何、工艺库、DEF 或 bump map。它通常通过
`Header.include` 引入 `.3dbv`；`.3dbv` 再引入 LEF、Liberty、DEF 和 `.bmap`。

## Parser 阶段

`ThreeDBlox::readDbx()` 首先调用：

```cpp
DbxParser parser(logger_);
DbxData data = parser.parseFile(dbx_file);
```

`DbxParser::parseFile()` 的步骤是：

1. 读取文本文件。
2. 调用 `BaseParser::parseDefines()` 处理文本级 `#!define` 宏替换。
3. 调用 `YAML::Load()` 解析 YAML。
4. 将 YAML section 解析到 `DbxData`。

核心数据结构定义在 `src/odb/src/3dblox/objects.h`：

```cpp
struct DbxData
{
  Header header;
  DesignDef design;
  std::map<std::string, ChipletInst> chiplet_instances;
  std::map<std::string, Connection> connections;
  std::map<std::string, PathAssertion> path_assertions;
};
```

路径解析由 `BaseParser::resolvePath()` 完成：相对路径会按当前正在解析的文件目录
展开；`include` 支持 `*` 通配符并展开为多个文件路径。

## include 递归读取

解析完 `.3dbx` 后，`readDbx()` 调用：

```cpp
readHeaderIncludes(data.header.includes);
```

`readHeaderIncludes()` 按文件后缀分派：

- 包含 `.3dbv`：调用 `readDbv(include)`。
- 包含 `.3dbx`：递归调用 `readDbx(include)`。

它使用 `read_files_` 记录绝对路径，避免重复读取。

需要注意的是，当前实现用 `include.find(".3dbv")` 和
`include.find(".3dbx")` 判断类型，因此本质上是基于字符串包含关系，而不是严格的
文件扩展名判断。

## 由 `.3dbv` 建立 chiplet master

`.3dbv` 描述 chiplet 定义。`readDbv()` 解析出 `DbvData` 后，会：

1. 设置或校验数据库 DBU 精度：
   - 如果 `db_->getDbuPerMicron() == 0`，使用 `.3dbv` header precision。
   - 如果数据库已有 DBU，则要求 `.3dbv` precision 不大于已有 DBU，且已有 DBU 是
     precision 的整数倍。
2. 继续处理 `.3dbv` 的 `Header.include`。
3. 对每个 `ChipletDef` 调用 `createChiplet()`。

`createChiplet()` 将 chiplet definition 落到 ODB：

- 读取 tech LEF：`lefin::createTechAndLib()`，创建或复用 `dbTech`。
- 读取普通 LEF：`lefin::createLib()`，创建 `dbLib` / `dbMaster`。
- 读取 Liberty：`sta_->readLiberty()`，供 STA 使用。
- 创建或复用 `dbChip`：
  - chip 类型由字符串映射为 `DIE`、`RDL`、`IP`、`SUBSTRATE`、`HIER`。
  - 已存在 chip 时会检查类型兼容性。
- 读取 DEF：`defin::readChip(..., chip, issue_callback=false)`，填充 chip 对应
  `dbBlock`，但暂不触发普通 DEF callback。
- 设置 chip 尺寸、厚度、shrink、TSV、scribe line、seal ring、offset 等属性。
- 对非 `HIER` 且尚无 block 的 chip 创建 blackbox `dbBlock`，并设置 die/core area。
- 为每个 region 调用 `createRegion()`。

`createRegion()` 创建 `dbChipRegion`：

- region side 映射为 `FRONT`、`BACK`、`INTERNAL`、`INTERNAL_EXT`。
- 如果指定 layer，则在 chip 的 `dbTech` 中查找 `dbTechLayer`。
- 将 region 多边形坐标合并为 `Rect`，并按 DBU 转换。
- 如果 region 指定 `.bmap`，读取 bump map 并创建 bump。

`createBump()` 对每条 bump map entry：

- 查找或创建 bump cell `dbInst`。
- 创建 `dbChipBump` 并绑定到 `dbChipRegion`。
- 将 bump 位置从 micron 转为 DBU，并按 chip offset 修正。
- 如果指定 net，查找或创建 `dbNet`，并连接 bump instance 的 first iterm。
- 如果指定 port，查找或创建 `dbBTerm`，并把 `dbChipBump` 关联到 bterm。

因此 `.3dbv` / `.bmap` 阶段主要建立 chiplet master、region、bump、block、net 和 port。

## 创建顶层 HIER chip

include 处理完后，`readDbx()` 调用：

```cpp
dbChip* chip = createDesignTopChiplet(data.design);
```

`createDesignTopChiplet()` 会：

- 创建类型为 `dbChip::ChipType::HIER` 的顶层 `dbChip`。
- 如果 `Design.external.verilog_file` 非空，在 chip 上创建字符串属性
  `verilog_file`。
- 调用 `db_->setTopChip(chip)`，使 `db_->getChip()` 返回这个顶层 chip。

这一步正是 `src/odb/test/read_3dbx.tcl` 中 `$db getChip` 不再为 `NULL` 的直接原因。

## 创建 chiplet 实例

`.3dbx` 的 `ChipletInst` 和 `Stack` 最终共同形成 `ChipletInst` 数据。

`createChipInst()` 的逻辑：

1. 用 `ChipletInst.reference` 调用 `db_->findChip()` 查找 master chiplet。
2. 在当前顶层 chip 上创建 `dbChipInst`：
   ```cpp
   dbChipInst::create(db_->getChip(), chip, chip_inst.name);
   ```
3. 处理 3D orient：
   - 先通过 `dup_orient_map` 归一化部分等价朝向字符串。
   - 再用 `dbOrientType3D::fromString()` 转成 ODB 3D 朝向类型。
4. 将 `Stack.loc` 和 `Stack.z` 从 micron 转为 DBU，设置到
   `dbChipInst::setLoc(Point3D{...})`。

此时 ODB 中形成了：

- 顶层 HIER `dbChip`。
- 多个 `dbChipInst`。
- 每个 `dbChipInst` 指向一个 master `dbChip`。
- 每个实例具备 3D 位置和朝向。

## 从 Verilog 构建 chip-level net

创建实例后，`readDbx()` 调用：

```cpp
buildChipNetsFromVerilog(chip, data);
```

这个步骤只在两个条件同时满足时执行：

- `sta_` 非空。
- `Design.external.verilog_file` 非空且文件存在。

流程如下：

1. 创建临时 `sta::ConcreteNetwork` 并复制当前 STA 状态。
2. 使用 `sta::VerilogReader` 读取顶层 Verilog。
3. 用 `data.design.name` link 顶层 module。
4. 遍历 Verilog 中的 net，创建同名 `dbChipNet`。
5. 遍历 net 上的 pin：
   - 跳过顶层 instance 自身 pin。
   - 用 Verilog instance 名查找顶层 `dbChipInst`。
   - 用 pin 的 port 名在 master chip block 中查找 `dbBTerm`。
   - 通过 `dbBTerm::getChipBump()` 找到 master bump。
   - 在实例化后的 region bump instances 中找到对应 `dbChipBumpInst`。
   - 将 bump instance 加入 `dbChipNet`。

所以顶层 Verilog 负责建立 chiplet 之间的逻辑 net，而 `.bmap` / bterm / bump
负责把逻辑端口映射到物理 bump。

## 创建 region 连接

`.3dbx` 的 `Connection` section 由 `createConnection()` 建立为 `dbChipConn`。

连接端点是字符串路径，例如：

```yaml
top: soc_inst_duplicate.regions.front_reg
bot: soc_inst.regions.front_reg
```

`resolvePath()` 负责解析路径：

- 路径分隔符是 `/`，用于表达层次化 chip instance 路径。
- 最后一段必须包含 `.regions.`，用于定位 region 名。
- 对每一级 instance，用 `curr_chip->findChipInst(inst_name)` 查找。
- 最终用 `curr_chip_inst->findChipRegionInst(region_name)` 找到实例化后的
  `dbChipRegionInst`。
- 特殊路径 `~` 返回 `nullptr`，用于表示 virtual/open endpoint。

随后创建连接：

```cpp
dbChipConn::create(connection.name,
                  db_->getChip(),
                  top_region_path,
                  top_region,
                  bottom_region_path,
                  bottom_region);
```

并将 `thickness` 从 micron 转为 DBU 后写入 `dbChipConn`。

## 计算顶层尺寸

所有实例和连接创建完成后，`readDbx()` 调用：

```cpp
calculateSize(chip);
```

`calculateSize()` 遍历顶层 chip 的所有 `dbChipInst`，合并每个实例的 3D cuboid，
再设置顶层 chip：

- `width = cuboid.xMax()`
- `height = cuboid.yMax()`
- `thickness = cuboid.dz()`

这一步让顶层 HIER chip 获得由所有 child chiplet 包围盒推导出的整体尺寸。

## 创建路径断言

如果 `.3dbx` 中包含 `Path` section，parser 会创建 `PathAssertion`：

- 名称必须是 `PathN`，例如 `Path1`，且 N 为正整数。
- 每条 entry 必须包含 `.regions.`。
- entry 可以用 `NOT ` 前缀表示否定。

`readDbx()` 为每个 path assertion 创建：

```cpp
dbChipPath* chip_path = dbChipPath::create(chip, assertion.name.c_str());
```

然后逐条解析 region path，并调用：

```cpp
chip_path->addEntry(path_insts, region_inst, entry.negated);
```

这类对象主要用于后续 3DBlox 检查或工具消费路径约束。

## 读取后的 observer 与 checker

`OpenRoad::read3Dbx()` 在 `parser.readDbx()` 后调用：

```cpp
db_->triggerPostRead3Dbx(db_->getChip());
```

`dbDatabase::triggerPostRead3Dbx()`：

- 遍历所有 `dbDatabaseObserver`。
- 调用每个 observer 的 `postRead3Dbx(chip)`。
- 调用 `constructUnfoldedModel()`。

当前已知 observer 行为：

- `dbSta::postRead3Dbx()` 目前是空实现，注释说明 chiplet timing 尚未准备好。
- GUI `MainWindow::postRead3Dbx()` 会发出 `chipLoaded(chip)` 信号。

随后 `OpenRoad::read3Dbx()` 调用 `check3DBlox()`，构造 `ThreeDBlox checker` 并运行
`Checker::check()`。checker 基于展开后的 3D model 做连接、重叠、region 使用、
bump 对齐、alignment marker 等检查。

## ODB 对象关系总结

读取 `example.3dbx` 后，ODB 中的主要对象关系如下：

```text
dbDatabase
  -> top dbChip: TopDesign, type=HIER
       -> dbChipInst: soc_inst
            -> master dbChip: SoC
            -> loc/orient from Stack
            -> dbChipRegionInst(s) from master dbChipRegion(s)
            -> dbChipBumpInst(s) from master dbChipBump(s)
       -> dbChipInst: soc_inst_duplicate
            -> master dbChip: SoC
       -> dbChipConn: soc_to_soc
            -> top/bottom dbChipRegionInst endpoints
       -> dbChipConn: soc_to_virtual
            -> one endpoint may be nullptr for "~"
       -> dbChipNet(s), if top Verilog exists and can be linked
       -> dbChipPath(s), if Path section exists

  -> master dbChip: SoC, type=DIE
       -> dbTech / dbLib / dbBlock from LEF/DEF
       -> dbChipRegion: front_reg, back_reg
       -> dbChipBump(s) from .bmap
       -> dbNet / dbBTerm mapping bump to logical names
```

## 关键源码位置

| 文件 | 作用 |
| --- | --- |
| `src/odb/test/read_3dbx.tcl` | 测试入口，验证读取后 top chip 非空 |
| `src/OpenRoad.tcl` | Tcl 命令定义、参数和文件检查 |
| `src/OpenRoad.i` | SWIG bridge，将 Tcl 命令转入 C++ |
| `src/OpenRoad.cc` | `OpenRoad::read3Dbx()` 入口、observer、checker |
| `src/odb/include/odb/3dblox.h` | `ThreeDBlox` API |
| `src/odb/src/3dblox/3dblox.cpp` | 读取 `.3dbx`、递归 include、建 ODB 的核心逻辑 |
| `src/odb/src/3dblox/dbxParser.cpp` | `.3dbx` YAML parser |
| `src/odb/src/3dblox/dbvParser.cpp` | `.3dbv` YAML parser |
| `src/odb/src/3dblox/baseParser.cpp` | 宏替换、路径解析、通用 YAML 提取 |
| `src/odb/src/3dblox/objects.h` | parser 中间数据结构 |
| `src/odb/src/3dblox/bmapParser.cpp` | bump map parser |
| `src/odb/src/3dblox/checker.cpp` | 读取后的 3DBlox checker |
| `src/odb/src/db/dbDatabase.cpp` | `triggerPostRead3Dbx()` |
| `src/dbSta/src/dbSta.cc` | dbSta observer，目前 `postRead3Dbx()` 为空 |
| `src/gui/src/mainWindow.cpp` | GUI observer，发出 chip loaded 信号 |

## 读入流程的几个实现细节

- `.3dbx` 顶层 `Design.name` 直接决定顶层 HIER `dbChip` 名称。
- `db_->setTopChip(chip)` 发生在 `createDesignTopChiplet()` 内。
- 坐标、厚度等用户输入单位是 micron，进入 ODB 时乘以
  `db_->getDbuPerMicron()` 并 round 成整数 DBU。
- `.3dbx` 的 `ChipletInst.external` 当前会被 parser 解析，但主读取流程中未直接使用
  这些 per-instance external 文件。
- `readDbx()` 会先处理 include，再创建顶层 HIER chip。因此 chiplet master 必须已经由
  include 中的 `.3dbv` 或递归 `.3dbx` 建好。
- 读取 DEF 时显式关闭普通 DEF callback，因为 `read_3dbx` 结束后会统一触发一次
  `postRead3Dbx()`。
- `readHeaderIncludes()` 的去重基于 `std::filesystem::absolute()`，可以避免常见重复
  include，但路径字符串形式差异仍可能影响去重效果。
