# IFP 模块对 ODB 的依赖梳理

## 概览

IFP（Init Floorplan，初始布局规划）负责芯片物理设计的最初几步：设定 Die 区域、生成标准单元行（Row）、创建布线轨道（Track）、插入 Tiecell。所有对物理数据库的读写均通过 `libodb` 完成。

**构建依赖**（`src/ifp/src/CMakeLists.txt`）：

```cmake
target_link_libraries(ifp PRIVATE odb)
```

**头文件依赖**：

| 头文件 | 提供内容 |
|--------|---------|
| `odb/db.h` | 所有 db 类（dbBlock, dbSite, dbRow, dbTechLayer, ...） |
| `odb/geom.h` | 几何类型（Rect, Polygon, Point） |
| `odb/dbTypes.h` | 枚举/类型定义 |
| `odb/util.h` | 工具函数（cutRows, makeSiteLoc） |
| `odb/PtrSetMap.h` | PtrSet 容器模板 |

---

## 按重要程度分层的 ODB 依赖

### 第一层：核心依赖

IFP 的任何功能都离不开这三个类。

#### `odb::dbBlock` — 设计模块

**角色**: IFP 的 `block_` 成员，构造时注入，是 IFP 与物理设计交互的唯一入口。所有操作——读设计状态、写布局结果——全部通过 `dbBlock` 完成。

**关键接口**（按功能分组）:

| 功能 | 调用的方法 |
|------|-----------|
| Die 区域管理 | `getDieArea()`, `setDieArea(Rect)`, `setDieArea(Polygon)` |
| Core 区域管理 | `getCoreArea()`, `setCoreArea()`, `computeCoreArea()` |
| 遍历设计数据 | `getInsts()`, `getRows()`, `getNets()`, `getBlockages()`, `getGroups()` |
| Track 管理 | `findTrackGrid(layer)` |
| 导航到上层对象 | `getDataBase()` → 进而访问 `dbTech`、`dbLib` |
| 单位转换 | `dbuToMicrons()`, `getDbUnitsPerMicron()` |

---

#### `odb::Rect` — 矩形

**角色**: IFP 中最基础的几何类型。几乎所有函数签名都用 `const odb::Rect&` 传参，用于表达 Die 区域、Core 区域、Row 包围盒等。

**关键接口**: 构造函数 `Rect(lx, ly, ux, uy)` / 聚合初始化 `{lx, ly, ux, uy}`；边界获取 `xMin()/yMin()/xMax()/yMax()`；尺寸 `dx()/dy()`、`area()`、`maxDXDY()`；空间关系 `contains(Rect)`。

---

#### `odb::dbSite` — 布局站点

**角色**: Row 生成的"模数"。Site 的宽高决定了 Core 区域每行放多少个单元、Core 区域总共排多少行。IFP 支持单一站点和混合高度（Hybrid Row）两种模式。

**关键接口**:

| 接口 | 用途 | 涉及功能 |
|------|------|---------|
| `getWidth()` / `getHeight()` | 决定行宽（sites/row）和行数（rows/core） | `makeUniformRows`, `makeHybridRows`, `makePolygonRowsScanline` |
| `hasRowPattern()` | 区分普通行和混合高度行，走不同的生成分支 | `makeRows` 中的分支判断 |
| `getRowPattern()` | 获取混合行的 pattern `[(site, orient), ...]` | `makeHybridRows`, `getOffset` |
| `getClass()` | 过滤 PAD site（不对 PAD 行做电压域切分和 Track 计算） | `updateVoltageDomain`, `makeTracksNonUniform` |

---

### 第二层：主要功能依赖

这三组分别对应 IFP 的三大输出：**Row**（行）、**Track**（轨道）、**面积校验**（前置检查）。

#### `odb::dbRow` — 布局行

**角色**: IFP 最核心的**输出**——在 Core 区域创建标准单元行。IFP 通过静态工厂方法创建和销毁 Row。

**关键接口**:

| 接口 | 用途 |
|------|------|
| `dbRow::create(block, name, site, origin_x, origin_y, orient, dir, num_sites, site_width)` | 在 `makeUniformRows`、`makeHybridRows`、`updateVoltageDomain`、`makeUniformRowsPolygon` 中创建新行 |
| `dbRow::destroy(row)` | 初始化前清除旧行（`prepareSitesAndClearRows`）、电压域切分时替换行（`updateVoltageDomain`） |
| `getBBox()` / `getSite()` / `getOrient()` | 电压域切分时读取行属性以重建分段行 |

---

#### `odb::dbTechLayer` + `odb::dbTrackGrid` — 布线层与轨道网格

**角色**: IFP 的 `makeTracks()` / `makeTracksNonUniform()` 自动为每个布线层创建 Track Grid，是 Row 之外的第二大输出。

**`dbTechLayer` 关键接口**:

| 接口 | 用途 |
|------|------|
| `getPitchX()` / `getPitchY()` | Track 间距 |
| `getOffsetX()` / `getOffsetY()` | Track 起始偏移 |
| `getMinWidth()` | 边界可用性检查（首末 track 是否在 die 内有效） |
| `getFirstLastPitch()` | 非零时走 LEF58_PITCH 非均匀 Track 路径 |
| `getType()` / `getRoutingLevel()` | 过滤非 routing 层和 level 0 |
| `getDirection()` | 非均匀 Track 仅支持水平层 |

**`dbTrackGrid` 关键接口**:

| 接口 | 用途 |
|------|------|
| `dbTrackGrid::create(block, layer)` | 为层创建新 grid |
| `dbTrackGrid::destroy(grid)` | `resetTracks()` 中清除旧 grid |
| `addGridPatternX/Y(origin, count, pitch)` | 写入 X/Y 方向的轨道模式 |

---

#### `odb::dbInst` + `odb::dbMaster` — 实例与单元定义

**角色**: 在 Row 生成之前，IFP 遍历所有实例计算设计面积（决定 Core 区域大小）、检查宏单元是否超出 Core 边界、收集实际使用的 Site 类型。

**`dbInst` 关键接口**:

| 接口 | 用途 |
|------|------|
| `getMaster()` | 通过实例访问其 Master 定义 |
| `dbInst::create(block, master, name)` | Tiecell 插入时创建新实例 |

**`dbMaster` 关键接口**:

| 接口 | 用途 |
|------|------|
| `getWidth()` / `getHeight()` | 计算面积、校验是否适配 Core |
| `isPad()` / `isCover()` / `isBlock()` / `isCoreAutoPlaceable()` | 过滤不参与面积计算的单元类型 |
| `getSite()` | 获取单元所用 Site（用于自动发现未显式指定的 site 类型） |
| `getSymmetryR90()` | R90 单元尺寸检查时使用 `max(width, height)` |

---

### 第三层：多边形支持

#### `odb::Polygon` + `odb::Point` — 多边形与点

**角色**: 支持非矩形（多边形）Die 和非矩形 Core 的 Row 生成。这是相对较新的功能，通过 scanline 算法裁剪位于多边形内部的 Row 段。

**关键接口**:

| 类型 | 接口 | 用途 |
|------|------|------|
| `Polygon` | `Polygon(points)` | 从顶点构造多边形 |
| `Polygon` | `getPoints()` | 获取顶点列表 |
| `Polygon` | `getEnclosingRect()` | 获取包围盒（确定 Row 生成范围） |
| `Point` | `Point(x, y)`, `x()`, `y()` | 顶点坐标 |

**涉及函数**: `makePolygonDie`, `makePolygonRows`, `makePolygonRowsScanline`, `intersectRowWithPolygon`, `makeUniformRowsPolygon`

---

### 第四层：次要功能依赖

以下类只在特定功能中使用，按功能分组。

#### Tiecell 插入（`insertTiecells`）

`insertTiecells` 根据 Liberty 中的 `function = "0"` 或 `"1"` 判断 tiecell 是 tie-low 还是 tie-high，查找所有 POWER/GROUND 网络并插入对应 tiecell 实例。

| ODB 类 | 角色 |
|--------|------|
| `dbMTerm` | 函数参数，指定 tiecell 的电源/地 pin |
| `dbNet` | 遍历查找 POWER/GROUND 网络 |
| `dbSigType` | 枚举 `GROUND` / `POWER` / `SIGNAL`，匹配和修改网络类型 |
| `dbITerm` | `connect(net)` 将 tiecell 实例的 pin 连到网络 |

#### 电压域处理（`updateVoltageDomain`）

识别 UPF 定义的 VOLTAGE_DOMAIN / POWER_DOMAIN group，将跨越域边界的 Row 切断，在每个子区域内重新生成行。

| ODB 类 | 角色 |
|--------|------|
| `dbGroup` | 遍历获取 group type 和 region |
| `dbGroupType` | 枚举 `VOLTAGE_DOMAIN` / `POWER_DOMAIN` |
| `dbRegion` | `getBoundaries()` 获取域边界多边形 |

#### Blockage 处理（`makeRows` / `makePolygonRowsScanline` 末尾）

| ODB 类/函数 | 角色 |
|-------------|------|
| `dbBlockage` | `getBBox()` 获取阻塞区域 |
| `dbBox` | 作为 getBBox 返回的包围盒类型 |
| `odb::cutRows()` | 工具函数，根据 Blockage 切割已生成的 Row |

---

### 第五层：辅助枚举与工具

这些在 IFP 中使用但职责单一、重要性较低。

| ODB 类型 | 用途 |
|----------|------|
| `dbOrientType` | Row 朝向：`R0`（正向/偶数行）、`MX`（翻转/奇数行）；Hybrid Row 模式下 `flipX()` 匹配翻转 pattern |
| `dbRowDir` | 恒为 `HORIZONTAL`（IFP 不创建垂直行） |
| `dbTech` | 通过 `block_->getDataBase()->getTech()` 访问：`hasManufacturingGrid()` / `getManufacturingGrid()` 用于坐标对齐；`getLayers()` 遍历层生成 Track |
| `dbDatabase` | 间接访问，通过 `block_->getDataBase()` 获取，不直接调用其方法 |
| `dbLib` | `findSite(name)` 按名称查找 Site（仅在 `findSite()` 中） |
| `dbSiteClass` | 枚举 `CORE` / `PAD`，用于过滤 |
| `dbTechLayerType` | 枚举 `ROUTING`，过滤布线层 |
| `dbTechLayerDir` | 枚举 `HORIZONTAL`，非均匀 Track 的前置检查 |
| `odb::makeSiteLoc()` | 将坐标对齐到 site 网格（电压域边界处理） |
| `PtrSet<dbSite>` | 作为 `flipped_sites` 参数的容器类型 |

---

## 依赖关系图

```
IFP (InitFloorplan.cc)
│
├── ★★★ 第一层：核心 — 每个函数都用到
│   ├── dbBlock     (设计数据唯一入口)
│   ├── Rect        (几何基础类型)
│   └── dbSite      (Row 生成的模数)
│
├── ★★☆ 第二层：主要输出 — Row / Track / 校验
│   ├── dbRow       (create/destroy — IFP 的核心输出)
│   ├── dbTrackGrid (create/destroy + addGridPattern — 第二大输出)
│   ├── dbTechLayer (pitch/offset/minWidth → Track 参数来源)
│   ├── dbInst      (遍历实例 → 面积/校验)
│   └── dbMaster    (单元尺寸/类型判断)
│
├── ★☆☆ 第三层：多边形支持
│   ├── Polygon     (多边形 Die/Core)
│   └── Point       (顶点坐标)
│
├── 第四层：次要功能
│   ├── [Tiecell]  dbMTerm, dbNet, dbSigType, dbITerm
│   ├── [电压域]    dbGroup, dbGroupType, dbRegion
│   └── [Blockage] dbBlockage, dbBox, odb::cutRows()
│
└── 第五层：辅助枚举/工具
    ├── dbOrientType, dbRowDir
    ├── dbTech, dbDatabase, dbLib
    ├── dbSiteClass, dbTechLayerType, dbTechLayerDir
    ├── odb::makeSiteLoc()
    └── PtrSet<dbSite>
```

---

## 涉及源文件

| 文件 | 说明 |
|------|------|
| `include/ifp/InitFloorplan.hh` | 主类声明（公开 API + 私有方法） |
| `src/InitFloorplan.cc` | 主实现（~1410 行），全部 ODB 交互逻辑 |
| `src/InitFloorplan.i` | SWIG Tcl 接口，Tcl 命令到 C++ 的映射 |
| `src/InitFloorplan-py.i` | SWIG Python 接口 |
| `src/MakeInitFloorplan.cc` | Tcl 初始化入口 |
| `src/CMakeLists.txt` | 构建配置 |
