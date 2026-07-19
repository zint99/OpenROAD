# PlacerBaseCommon::init() 初始化流程文档

> 源文件：`src/gpl/src/placerBase.cpp`（第 752–998 行）
> 模块：OpenROAD GPL（全局布局器 Global Placement）主数据结构初始化
> 调用时机：在 `PlacerBaseCommon` 构造函数末尾被直接调用，负责把 OpenDB 中的物理设计数据转换为 GPL 内部使用的 `Instance` / `Pin` / `Net` / `Die` 数据结构。

---

## 0. 总览

`init()` 的核心任务是从 `odb::dbBlock` 中提取 die/core 几何、site 尺寸，并把数据库中的实例、网络、引脚映射成 GPL 自有的轻量对象，同时建立三者之间的双向索引（`instMap_` / `pinMap_` / `netMap_` 以及 `insts_` / `pins_` / `nets_` 指针列表）。成功完成后，上层 `PlacerBase` 即可基于这些结构进行区域划分与布局。

整体执行顺序：

```
入口/日志 → 站点与Die/Core → 填充实例 → 引脚密度统计 → 按密度扩展
        → 构建实例索引 → 填充网络/引脚 → 构建引脚索引
        → 实例挂引脚 → 网络挂引脚
```

---

## 1. 入口与日志（第 752–757 行）

构造函数 `PlacerBaseCommon(...)` 在初始化列表中设置 `pbVars_` 后，于函数体末尾调用 `init()`。

`init()` 首先打印初始化横幅与 DBU（Database Units Per Micron，每微米数据库单位数）：

```cpp
log_->info(GPL, 1, "---- Initialize GPL Main Data Structures");
log_->info(GPL, 2, "DBU: {}", db_->getTech()->getDbUnitsPerMicron());
```

随后取得设计的访问入口：

```cpp
dbBlock* block = db_->getChip()->getBlock();
```

DBU 是后续所有坐标与面积换算（如 `dbuToMicrons`、`dbuAreaToMicrons`）的基准。

---

## 2. 站点与 Die/Core 几何（第 759–796 行）

### 2.1 查找 site

遍历 `block->getRows()`，跳过 `PAD` 类 site，取第一个非 PAD site：

```cpp
odb::dbSite* db_site = nullptr;
for (auto* row : block->getRows()) {
  if (row->getSite()->getClass() != odb::dbSiteClass::PAD) {
    db_site = row->getSite();
    break;
  }
}
if (db_site == nullptr) {
  log_->error(GPL, 305, "Unable to find a site");
}
```

### 2.2 取 die / core 矩形并校验

```cpp
odb::Rect core_rect = block->getCoreArea();
odb::Rect die_rect  = block->getDieArea();
if (!die_rect.contains(core_rect)) {
  log_->error(GPL, 118, "core area outside of die.");
}
die_ = Die(die_rect, core_rect);
```

- 若 core 不在 die 内 → 报错 `GPL 118` 并终止。
- 用两个矩形构造 `Die` 对象，记录 die 与 core 的四角坐标。

### 2.3 记录 site 尺寸

```cpp
siteSizeX_ = db_site->getWidth();
siteSizeY_ = db_site->getHeight();
```

随后打印 `SiteSize` 与 `CoreBBox`（换算为微米）。

---

## 3. 填充真实实例到 `instStor_`（第 798–835 行）

遍历 `block->getInsts()`，为每一个"真实"实例构造 GPL 的 `Instance` 对象：

```cpp
dbSet<dbInst> db_insts = block->getInsts();
instStor_.reserve(db_insts.size());
insts_.reserve(instStor_.size());
for (dbInst* db_inst : db_insts) {
  auto type = db_inst->getMaster()->getType();
  if (!type.isCore() && !type.isBlock()) {
    continue;                       // 跳过非 core/非 block（如纯供电单元）
  }
  Instance temp_inst(db_inst, this, log_);
  ...
}
```

对每个候选实例：

1. **尺寸校验**
   - 实例高度 > core 高度 → 报错 `GPL 119`。
   - 实例宽度 > core 宽度 → 报错 `GPL 120`。

2. **固定实例向外对齐（snapOutward）**
   ```cpp
   if (temp_inst.isFixed()) {
     temp_inst.snapOutward(core_rect.ll(), siteSizeX_, siteSizeY_);
   }
   ```
   固定实例的包围盒被向外对齐到最近的 site 边界。原因：部分重叠的 site 不可用，向外 snap 可确保该 site 被算作"完全占用"，避免后续密度计算把它误当空闲。

3. **宏单元面积累计**
   ```cpp
   if (temp_inst.dy() > siteSizeY_ * 6) {
     macroInstsArea_ += temp_inst.getArea();
   }
   ```
   高度超过 6 行（或在 LEF 中标记为 BLOCK）的实例被视作宏单元。

> 注：`Instance` 构造函数内部（`placerBase.cpp:59-82`）会根据 `master->getType().isBlock()` 或"高度 > 6 行"判定 `is_macro_`，并对后者打印 `GPL 134` 警告。

---

## 4. 引脚密度统计与平均密度（第 837–884 行）

为后续的"按密度扩展"准备统计信息。

### 4.1 信号引脚计数 lambda

```cpp
auto count_signal_pins = [](const Instance& inst) -> int {
  return std::ranges::count_if(
      inst.dbInst()->getITerms(),
      [](odb::dbITerm* iterm) { return !iterm->getSigType().isSupply(); });
};
```

### 4.2 遍历统计

```cpp
int total_signal_pins = 0;
int64_t movable_area = 0;
int64_t total_area   = 0;
for (const auto& inst : instStor_) {
  if (inst.isInstance()) {
    total_area += inst.getArea();
    if (!inst.isFixed()) {
      total_signal_pins += count_signal_pins(inst);
      movable_area      += inst.getArea();
    }
  }
}
```

### 4.3 平均密度

```cpp
double avg_density
    = (movable_area > 0)
          ? static_cast<double>(total_signal_pins) / movable_area
          : 0.0;
```

并打印 `Movable instances area` 与 `Total instances area`。debug 模式（`extendPinDensity`）下还会打印每引脚平均面积等细节。

---

## 5. 按引脚密度扩展实例尺寸（第 886–911 行）

目的：让引脚多的可动单元在布局中占据更大"占位"，缓解后续布线拥塞。该步骤**可被 `pbVars_.disablePinDensityAdjust` 关闭**。

```cpp
int64_t total_adjustment_area = 0;
if (!pbVars_.disablePinDensityAdjust) {
  for (auto& inst : instStor_) {
    if (!inst.isFixed() && inst.isInstance()) {
      int pin_count = count_signal_pins(inst);
      if (pin_count > 2 && avg_density > 0.0) {
        double target_area = static_cast<double>(pin_count) / avg_density;
        double scale
            = std::sqrt(target_area / static_cast<double>(inst.getArea()));
        if (scale > 1.2)       scale = 1.2;        // 上限，避免可布线性过度膨胀
        else if (scale < 0.95) scale = 0.95;       // 下限
        total_adjustment_area += inst.extendSizeByScale(scale, log_);
      }
    }
  }
}
```

- 仅对 `pin_count > 2` 的非固定实例生效。
- 缩放系数 `scale = sqrt(target_area / area)`，限制在 **[0.95, 1.2]** 区间。
- `extendSizeByScale` 以实例中心为基准缩放包围盒。
- 最后打印 `Pin density area adjust`。

---

## 6. 构建实例索引与可动列表（第 913–922 行）

```cpp
instMap_.reserve(instStor_.size());
insts_.reserve(instStor_.size());
for (auto& pb_inst : instStor_) {
  instMap_[pb_inst.dbInst()] = &pb_inst;   // dbInst* -> Instance*
  insts_.push_back(&pb_inst);              // 全部实例指针
  if (!pb_inst.isFixed()) {
    placeInsts_.push_back(&pb_inst);       // 仅可动实例
  }
}
```

| 容器 | 类型 | 含义 |
|---|---|---|
| `instMap_` | `unordered_flat_map<dbInst*, Instance*>` | 数据库实例 → GPL 实例 |
| `insts_` | `vector<Instance*>` | 全局实例指针列表 |
| `placeInsts_` | `vector<Instance*>` | 可放置（非固定）实例列表 |

---

## 7. 填充网络与引脚到 `netStor_` / `pinStor_`（第 924–954 行）

遍历 `block->getNets()`，仅处理 **SIGNAL / CLOCK** 信号网（VDD/VSS/reset 等供电网被 escape 跳过）：

```cpp
dbSet<dbNet> db_nets = block->getNets();
netStor_.reserve(db_nets.size());
for (dbNet* db_net : db_nets) {
  dbSigType net_type = db_net->getSigType();
  if (net_type == dbSigType::SIGNAL || net_type == dbSigType::CLOCK) {
    Net temp_net(db_net, pbVars_.skipIoMode);
    netStor_.push_back(temp_net);
    Net* temp_net_ptr = &netStor_[netStor_.size() - 1];
    netMap_[db_net] = temp_net_ptr;

    for (dbITerm* iTerm : db_net->getITerms()) {
      Pin temp_pin(iTerm);
      temp_pin.setNet(temp_net_ptr);
      temp_pin.setInstance(dbToPb(iTerm->getInst()));
      pinStor_.push_back(temp_pin);
    }
    if (!pbVars_.skipIoMode) {
      for (dbBTerm* bTerm : db_net->getBTerms()) {
        Pin temp_pin(bTerm, log_);
        temp_pin.setNet(temp_net_ptr);
        pinStor_.push_back(temp_pin);
      }
    }
  }
}
```

要点：
- `skipIoMode` 为真时跳过顶层端口（BTerm）引脚的创建。
- 每个 `Pin` 在创建时即关联其所属 `Net` 与 `Instance`（`dbToPb` 经 `instMap_` 查表）。

---

## 8. 构建引脚索引（第 956–966 行）

```cpp
pins_.reserve(pinStor_.size());
pinMap_.reserve(pinStor_.size());
for (auto& pb_pin : pinStor_) {
  if (pb_pin.isITerm()) {
    pinMap_[pb_pin.getDbITerm()] = &pb_pin;
  } else if (pb_pin.isBTerm()) {
    pinMap_[pb_pin.getDbBTerm()] = &pb_pin;
  }
  pins_.push_back(&pb_pin);
}
```

| 容器 | 类型 | 含义 |
|---|---|---|
| `pinMap_` | `unordered_flat_map<dbObject*, Pin*>` | 数据库引脚（ITerm/BTerm）→ GPL Pin |
| `pins_` | `vector<Pin*>` | 全局引脚指针列表 |

---

## 9. 实例挂载引脚（第 968–983 行）

把 Pin 反向挂到其所属 Instance（供电引脚在 `pinMap_` 中无记录，被安全跳过）：

```cpp
for (auto& pb_inst : instStor_) {
  if (!pb_inst.isInstance()) continue;
  for (dbITerm* iTerm : pb_inst.dbInst()->getITerms()) {
    Pin* cur_pin = dbToPb(iTerm);   // 经 pinMap_ 查表
    if (cur_pin) {
      pb_inst.addPin(cur_pin);
    }
  }
}
```

---

## 10. 网络挂载引脚并构建 `nets_`（第 985–997 行）

```cpp
nets_.reserve(netStor_.size());
for (auto& pb_net : netStor_) {
  for (dbITerm* iTerm : pb_net.getDbNet()->getITerms()) {
    pb_net.addPin(dbToPb(iTerm));
  }
  if (!pbVars_.skipIoMode) {
    for (dbBTerm* bTerm : pb_net.getDbNet()->getBTerms()) {
      pb_net.addPin(dbToPb(bTerm));
    }
  }
  nets_.push_back(&pb_net);
}
```

至此，`Instance ↔ Pin ↔ Net` 三者的双向关联全部建立完成，主数据结构初始化结束。

---

## 关键不变量与注意事项

1. **宏判定一致性**：`is_macro_` 在 `Instance` 构造函数（依据 LEF type 或高度 > 6 行）与 `init()` 中按 `dy() > siteSizeY_ * 6` 累计 `macroInstsArea_` 两处逻辑保持一致。
2. **固定实例 snapping**：仅固定实例执行 `snapOutward`，目的是让部分重叠 site 被计为完全占用。
3. **供电网 escape**：VDD/VSS/reset 等 `dbSigType` 既不在网络构建中处理，其引脚也不会挂到实例上。
4. **可关闭的扩展**：`disablePinDensityAdjust` 为真时跳过第 5 步；`skipIoMode` 为真时跳过所有 BTerm 相关处理。
5. **索引三件套**：`instMap_` / `pinMap_` / `netMap_` 提供 db 对象 → GPL 对象的 O(1) 查表（`dbToPb` 系列方法），是后续所有反查的基础。

---

## 附：相关成员容器一览（`placerBase.h`）

| 成员 | 声明 | 角色 |
|---|---|---|
| `die_` | `Die` | die/core 几何 |
| `siteSizeX_ / siteSizeY_` | `int` | site 尺寸 |
| `instStor_` | `vector<Instance>` | 实例存储（拥有所有权） |
| `pinStor_` | `vector<Pin>` | 引脚存储（拥有所有权） |
| `netStor_` | `vector<Net>` | 网络存储（拥有所有权） |
| `insts_ / pins_ / nets_` | `vector<指针>` | 全局指针索引 |
| `placeInsts_` | `vector<Instance*>` | 可动实例 |
| `instMap_ / pinMap_ / netMap_` | `unordered_flat_map` | db 对象 → GPL 对象 |
| `macroInstsArea_` | `int64_t` | 宏单元面积累计 |
| `pbVars_` | `PlacerBaseVars` | 放置选项（padLeft/Right、skipIoMode、disablePinDensityAdjust） |
