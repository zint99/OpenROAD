# Global Placement 新人入门规划

本文面向刚开始接触 OpenROAD `src/gpl` 模块的开发者。目标不是把每个公式和实现细节一次性讲完，而是给出一条可以执行的学习路径：先跑通模块，再理解数据结构和主循环，最后能安全地修 bug、加选项、补测试和参与评审。

## 目标读者和学习成果

完成本规划后，新人应能做到：

- 解释 `global_placement` 从 Tcl 命令到 C++ 算法核心的调用链。
- 读懂 `PlaceOptions`、`PlacerBase`、`NesterovBase`、`NesterovPlace` 的职责边界。
- 判断一个改动会影响普通 placement、timing-driven placement、routability-driven placement、incremental placement、GPU backend 还是 debug/GUI 路径。
- 为 GPL 改动添加或更新回归测试，并同时完成 CMake 和 Bazel 注册。
- 在修 bug 时追到数据创建点，而不是只在输出或序列化位置打补丁。

## 模块概览

`gpl` 是 OpenROAD 的 global placement 模块，核心基于 RePlAce/ePlace 系列 analytic nonlinear placement 方法。默认流程包括：

1. Tcl 命令解析用户参数。
2. 从 OpenDB 读取 design、row、instance、pin、net、region 等信息。
3. 可选执行 initial placement，用 BiCGSTAB 解稀疏线性方程得到初始位置。
4. 执行 Nesterov global placement，在 wirelength force 和 density force 之间迭代平衡。
5. 可选执行 timing-driven、routability-driven、virtual CTS、incremental placement。
6. 将最终坐标写回 OpenDB。

主入口文件：

- 用户文档：[../README.md](../README.md)
- Tcl 命令：[../src/replace.tcl](../src/replace.tcl)
- SWIG 绑定：[../src/replace.i](../src/replace.i)
- 公开 C++ API：[../include/gpl/Replace.h](../include/gpl/Replace.h)
- 主流程实现：[../src/replace.cpp](../src/replace.cpp)

## 前置知识

建议先补齐以下背景，再深入改代码：

- OpenROAD 基础：LEF/DEF、OpenDB block/inst/net/term、row/site、placement status。
- STA 基础：slack、critical path、resizer、clock latency。
- Placement 基础：HPWL、target density、overflow、bin grid、filler cell、macro/fixed cell。
- 数值方法：Nesterov acceleration、line search/backtracking、weighted-average wirelength、Poisson/FFT density solve。
- 工程工具：Tcl、C++17、Eigen sparse matrix、OpenMP、GTest、CMake、Bazel。

推荐阅读顺序：

1. RePlAce 论文，了解 GPL 算法来源。
2. ePlace/ePlace-MS 相关论文，了解 electrostatic density force、mixed-size placement 和 Nesterov 主循环。
3. OpenROAD `README.md` 中 GPL command options。
4. `test/simple01.tcl`、`test/simple01-td.tcl`、`test/simple02-rd.tcl` 等小测试。

## 快速上手

从 OpenROAD 仓库根目录运行小测试，优先选择一个最小 Tcl regression：

```sh
ctest -R '^gpl\.simple01\.tcl$' --output-on-failure
bazel test //src/gpl/test:simple01
```

如果只想手动观察一次 placement，可从仓库根目录运行：

```sh
./build/src/openroad src/gpl/test/simple01.tcl
```

常用实验参数：

```tcl
global_placement -skip_initial_place -init_density_penalty 0.01
global_placement -density 0.75 -overflow 0.2
global_placement -timing_driven
global_placement -routability_driven
global_placement_debug -pause 100 -update 1 -initial -draw_bins
```

## 架构分层

| 层级 | 关键文件 | 主要职责 |
| --- | --- | --- |
| 命令和绑定层 | [../src/replace.tcl](../src/replace.tcl), [../src/replace.i](../src/replace.i), [../src/replace-py.i](../src/replace-py.i) | 定义 Tcl/Python 命令，把 key/flag 转成 `PlaceOptions`，调用 `Replace` |
| 公开 API 和编排层 | [../include/gpl/Replace.h](../include/gpl/Replace.h), [../src/replace.cpp](../src/replace.cpp) | 保存 OpenDB/STA/RSZ/GRT/logger 指针，管理 initial place、Nesterov place、incremental、MBFF、debug |
| OpenDB 到 placement model | [../src/placerBase.h](../src/placerBase.h), [../src/placerBase.cpp](../src/placerBase.cpp) | 构建 `Instance`、`Pin`、`Net`、`Die`，按 top-level 和 region/group 切分 placement domain |
| Initial placement | [../src/initialPlace.h](../src/initialPlace.h), [../src/initialPlace.cpp](../src/initialPlace.cpp), [../src/solver.cpp](../src/solver.cpp) | 建立稀疏矩阵，用 BiCGSTAB 给 movable instances 一个较好的起点 |
| Nesterov 数据模型 | [../src/nesterovBase.h](../src/nesterovBase.h), [../src/nesterovBase.cpp](../src/nesterovBase.cpp) | 管理 `GCell/GPin/GNet/BinGrid/Bin/filler`，计算 density、overflow、gradient 和 DB callback state |
| Nesterov 主循环 | [../src/nesterovPlace.h](../src/nesterovPlace.h), [../src/nesterovPlace.cpp](../src/nesterovPlace.cpp) | 迭代、backtracking、convergence、divergence revert、timing/routability hook、最终写回 DB |
| Wirelength 和 density backend | [../src/hpwl.cpp](../src/hpwl.cpp), [../src/wirelengthGradient.cpp](../src/wirelengthGradient.cpp), [../src/densityGradient.cpp](../src/densityGradient.cpp), [../src/fft.cpp](../src/fft.cpp) | CPU/GPU strategy backend，计算 HPWL、WA wirelength gradient、density gradient、FFT |
| Timing/routability 扩展 | [../src/timingBase.h](../src/timingBase.h), [../src/timingBase.cpp](../src/timingBase.cpp), [../src/routeBase.h](../src/routeBase.h), [../src/routeBase.cpp](../src/routeBase.cpp), [../src/clockBase.h](../src/clockBase.h) | 调用 resizer/STA/GRT/RUDY，做 net reweight、cell inflation、virtual CTS |
| Debug 和 GUI | [../src/AbstractGraphics.h](../src/AbstractGraphics.h), [../src/graphicsImpl.cpp](../src/graphicsImpl.cpp), [../src/graphicsNone.cpp](../src/graphicsNone.cpp) | 迭代可视化、图片/GIF、选中实例调试 |
| MBFF | [../src/mbff.h](../src/mbff.h), [../src/mbff.cpp](../src/mbff.cpp) | `cluster_flops` 命令和 multi-bit flop clustering |
| 测试和消息 | [../test/CMakeLists.txt](../test/CMakeLists.txt), [../test/BUILD](../test/BUILD), [../messages.txt](../messages.txt) | Tcl/Python/C++ regression、GPU 条件测试、日志消息文档检查 |

## 主调用链

普通 `global_placement` 的核心链路：

```text
global_placement Tcl command
  -> gpl::replace_initial_place_cmd / gpl::replace_nesterov_place_cmd
  -> Replace::doInitialPlace
     -> PlacerBaseCommon / PlacerBase
     -> InitialPlace::doBicgstabPlace
  -> Replace::doNesterovPlace
     -> Replace::initNesterovPlace
        -> NesterovBaseCommon
        -> one NesterovBase per placement region
        -> RouteBase / TimingBase / optional ClockBase
        -> NesterovPlace
     -> NesterovPlace::doNesterovPlace
        -> update wirelength and density gradients
        -> update coordinates and overflow
        -> optional timing-driven net reweight or repair
        -> optional routability-driven inflation
        -> update OpenDB
```

增量模式走 `Replace::doIncrementalPlace`。它会先锁住已放置实例，粗放未放置对象，再解锁并按用户 overflow 目标完成第二阶段 placement。

## 核心概念

### `PlaceOptions`

`PlaceOptions` 是所有命令参数进入 C++ 后的统一配置对象，定义在 [../include/gpl/Replace.h](../include/gpl/Replace.h)。新增命令参数时通常要同时更新：

- `src/replace.tcl` 的 `define_cmd_args`、`parse_key_args`。
- `src/replace.i` 的 `getOptions()`。
- `include/gpl/Replace.h` 的 `PlaceOptions` 默认值。
- `PlaceOptions::validate()` 的范围检查。
- `README.md` 和必要测试。

### `PlacerBaseCommon` 和 `PlacerBase`

`PlacerBaseCommon` 保存不按 region 切分的全局实例、pin、net 映射。`PlacerBase` 表示一个 placement domain，通常是 top-level core 或某个 power domain/group。

重点关注：

- `Instance` 区分 movable、fixed、macro、dummy、locked。
- `Pin` 同时支持 `dbITerm` 和 `dbBTerm`。
- `Net` 负责 box 和 HPWL 基础信息。
- dummy instances 用于表达不可用 site 或 blockage。
- `pbVec_` 中可能有多个 region，top-level 没有 placeable instances 时会被移除。

### `NesterovBaseCommon` 和 `NesterovBase`

`NesterovBaseCommon` 保存全局 `GCell/GPin/GNet` 和 OpenDB 到 NB 的映射，是 wirelength 计算的中心。`NesterovBase` 是 per-region 的 density 计算中心，持有 bin grid、fillers、region density state 和迭代坐标。

理解时先抓住三类坐标：

- OpenDB 坐标：最终写回 design。
- `GCell` normal coordinates：当前实例或 filler 几何位置。
- density coordinates：用于 density spreading 和 bin overlap 的虚拟尺寸/位置。

### Wirelength 和 density

GPL 的目标函数大体由两部分组成：

- wirelength force：weighted-average 模型近似 HPWL。
- density force：把 cell 面积投影到 bin grid，通过 FFT/Poisson 求 electrostatic field。

`NesterovPlace::updateWireLengthCoef()` 会根据 overflow 动态调整 wirelength coefficient。`NesterovBase::updateGradients()` 会把 wirelength gradient 和 density gradient 按 density penalty 合成总梯度。

### Timing-driven placement

`-timing_driven` 由 `TimingBase` 管理。它根据 overflow threshold 触发 timing iteration，调用 STA/resizer 获取 slack，再对关键 nets 加权。相关参数包括：

- `-timing_driven_net_reweight_overflow`
- `-timing_driven_net_weight_max`
- `-timing_driven_nets_percentage`
- `-keep_resize_below_overflow`
- `-timing_driven_repair_timing`

如果启用 `-virtual_cts`，`ClockBase` 会在 timing iteration 前设置虚拟 clock insertion delay，placement 结束前移除。

### Routability-driven placement

`-routability_driven` 由 `RouteBase` 管理。默认使用 RUDY 估计 congestion，也可用 `-routability_use_grt` 调用 GRT。拥塞 tile 会触发 cell area inflation，并通过 snapshot/revert 保留更好的 routing congestion 状态。

关键参数：

- `-routability_target_rc_metric`
- `-routability_check_overflow`
- `-routability_snapshot_overflow`
- `-routability_max_density`
- `-routability_inflation_ratio_coef`
- `-routability_max_inflation_ratio`
- `-routability_rc_coefficients`

### CPU/GPU backend

CPU backend 总是编译。`ENABLE_GPU=ON` 时会额外编译 `src/gpu/`，并通过 runtime 选择 GPU path。backend strategy 接口包括：

- [../src/hpwlBackend.h](../src/hpwlBackend.h)
- [../src/wirelengthGradientBackend.h](../src/wirelengthGradientBackend.h)
- [../src/densityGradientBackend.h](../src/densityGradientBackend.h)
- [../src/fftBackend.h](../src/fftBackend.h)
- [../src/backendContext.h](../src/backendContext.h)

改 GPU 相关路径时要特别注意 CPU/GPU 一致性、host/device state 同步、multi-region case 和 golden test 的 deterministic 要求。

## 四周入门计划

### 第 0 周：环境和最小闭环

目标：能独立跑 GPL 小测试，并知道失败时先看哪里。

任务：

- 从仓库根目录跑 `ctest -R '^gpl\.simple01\.tcl$' --output-on-failure`。
- 跑 `bazel test //src/gpl/test:simple01`。
- 阅读 [../README.md](../README.md) 的 `global_placement` 参数说明。
- 阅读 [../test/simple01.tcl](../test/simple01.tcl)、[../test/simple01.ok](../test/simple01.ok)。
- 打开 [../messages.txt](../messages.txt)，了解 GPL 日志编号的维护方式。

交付物：

- 一页个人笔记：如何运行 GPL 单测、日志中 HPWL/overflow/density 的含义、常见失败信息。

### 第 1 周：命令层和主流程

目标：能从 Tcl 参数追到 `Replace` 的 C++ 调用。

任务：

- 阅读 [../src/replace.tcl](../src/replace.tcl)，重点看 `global_placement`、`cluster_flops`、`global_placement_debug`、`placement_cluster`。
- 阅读 [../src/replace.i](../src/replace.i)，理解 `getOptions()` 如何填充 `PlaceOptions`。
- 阅读 [../include/gpl/Replace.h](../include/gpl/Replace.h)，标注所有 mode flag。
- 阅读 [../src/replace.cpp](../src/replace.cpp)，画出 `doInitialPlace()`、`doNesterovPlace()`、`doIncrementalPlace()` 的调用图。
- 对比 `test/simple01.tcl`、`test/simple06.tcl`、`test/incremental01.tcl`，理解 `-skip_initial_place`、`-skip_nesterov_place`、`-incremental` 的行为差异。

实践任务：

- 新增一个本地实验 Tcl，分别跑默认 placement 和 `-skip_initial_place`，观察 HPWL、overflow、迭代数变化。实验文件不需要提交。

交付物：

- 能口头说明：`global_placement` 为什么会先 initial place，再 Nesterov place；`-incremental` 为什么是独立入口。

### 第 2 周：数据模型和 initial placement

目标：理解 OpenDB design 如何变成 GPL 内部对象。

任务：

- 阅读 [../src/placerBase.h](../src/placerBase.h) 中 `Instance`、`Pin`、`Net`、`Die`、`PlacerBaseCommon`、`PlacerBase`。
- 阅读 [../src/placerBase.cpp](../src/placerBase.cpp) 的初始化逻辑和日志输出。
- 阅读 [../src/initialPlace.h](../src/initialPlace.h)、[../src/initialPlace.cpp](../src/initialPlace.cpp)，理解 BiCGSTAB initial placement 的矩阵构建。
- 阅读 [../test/core01.tcl](../test/core01.tcl)、[../test/macro01.tcl](../test/macro01.tcl)、[../test/region01.tcl](../test/region01.tcl)，观察 core、macro、region 场景。

实践任务：

- 用一个小测试跟踪 `PlacerBaseCommon::printInfo()` 的日志，确认 movable、fixed、dummy、macro 面积统计。
- 找一个包含 region 的测试，确认 `pbVec_` 和 `nbVec_` 中对象数量。

交付物：

- 能解释 `PlacerBaseCommon` 与 `PlacerBase` 的区别，以及为什么 multi-region placement 需要多个 `NesterovBase`。

### 第 3 周：Nesterov 主循环

目标：能读懂一次迭代里发生了什么。

任务：

- 阅读 [../src/nesterovBase.h](../src/nesterovBase.h) 的 `GCell`、`GPin`、`GNet`、`Bin`、`BinGrid`、`NesterovBaseCommon`、`NesterovBase`。
- 阅读 [../src/nesterovPlace.cpp](../src/nesterovPlace.cpp) 的 `doNesterovPlace()`、`doBackTracking()`、`updateNextIter()`、`isConverged()`、`isDiverged()`。
- 阅读 [../src/wirelengthGradient.cpp](../src/wirelengthGradient.cpp)、[../src/densityGradient.cpp](../src/densityGradient.cpp)、[../src/fft.cpp](../src/fft.cpp)。
- 跑 `test/diverge01.tcl` 和 `test/convergence01.tcl`，理解 divergence/revert 和 convergence 判断。

实践任务：

- 在本地构建里临时打开一个 debugPrint 分类，观察 `wireLengthCoef`、`densityPenalty`、`overflow` 的变化。不要提交调试输出。
- 用 `global_placement_debug` 生成一次图片或 GIF，确认可视化路径可用。

交付物：

- 能画出一轮 Nesterov iteration 的顺序：backtracking、gradient、coordinate update、overflow update、timing/routability hook、convergence check。

### 第 4 周：专题深入和首次贡献

目标：选择一个专题做可提交的小改动。

专题路线：

- Timing-driven：读 [../src/timingBase.cpp](../src/timingBase.cpp)、`test/simple01-td*.tcl`、`test/simple01-vcts*.tcl`。
- Routability-driven：读 [../src/routeBase.cpp](../src/routeBase.cpp)、`test/simple02-rd.tcl`、`test/simple04-rd.tcl`。
- GPU backend：读 `src/gpu/`、backend headers、`test/fft_gpu_test.cc`、`test/wl_gpu_test.cc`、`test/region01_gpu*.tcl`。
- MBFF：读 [../src/mbff.cpp](../src/mbff.cpp)、`test/mbff_hier.tcl`、`test/mbff_test.cpp`。
- Command/API：加一个小参数、校验或错误路径测试。

首次贡献建议：

- 修一个 option validation、错误消息、文档不一致或小型 corner case。
- 优先加 focused regression，不做大规模重构。
- 改 C++ 后运行 `clang-format -i <files>`，不要格式化 `*.i` 文件。
- 新增测试必须同时注册 [../test/CMakeLists.txt](../test/CMakeLists.txt) 和 [../test/BUILD](../test/BUILD)。

## 文件地图

| 文件 | 新人阅读优先级 | 说明 |
| --- | --- | --- |
| [../README.md](../README.md) | 高 | 用户视角的命令、参数和 mode 说明 |
| [../src/replace.tcl](../src/replace.tcl) | 高 | Tcl command 入口和参数解析 |
| [../src/replace.i](../src/replace.i) | 高 | Tcl 到 C++ 的 SWIG 绑定 |
| [../include/gpl/Replace.h](../include/gpl/Replace.h) | 高 | 公共 API、`PlaceOptions`、`Replace` 生命周期 |
| [../src/replace.cpp](../src/replace.cpp) | 高 | 模块编排和 mode 切换 |
| [../src/placerBase.h](../src/placerBase.h) | 高 | OpenDB 到 GPL 数据模型 |
| [../src/placerBase.cpp](../src/placerBase.cpp) | 高 | 数据结构初始化、面积统计、row/site 处理 |
| [../src/initialPlace.cpp](../src/initialPlace.cpp) | 中 | 初始位置求解 |
| [../src/nesterovBase.h](../src/nesterovBase.h) | 高 | Nesterov 数据结构声明 |
| [../src/nesterovBase.cpp](../src/nesterovBase.cpp) | 高 | density、filler、gradient、callback、convergence 状态 |
| [../src/nesterovPlace.cpp](../src/nesterovPlace.cpp) | 高 | 主迭代、timing/routability hook、revert |
| [../src/timingBase.cpp](../src/timingBase.cpp) | 中 | timing-driven net reweight 和 resizer 调用 |
| [../src/routeBase.cpp](../src/routeBase.cpp) | 中 | RUDY/GRT congestion 和 inflation |
| [../src/clockBase.cpp](../src/clockBase.cpp) | 中 | virtual CTS |
| [../src/hpwl.cpp](../src/hpwl.cpp) | 中 | HPWL backend factory 和 CPU 实现 |
| [../src/wirelengthGradient.cpp](../src/wirelengthGradient.cpp) | 中 | WA wirelength gradient backend |
| [../src/densityGradient.cpp](../src/densityGradient.cpp) | 中 | density gradient backend |
| [../src/fft.cpp](../src/fft.cpp) | 中 | FFT backend factory |
| [../src/gpu/](../src/gpu) | 低到高 | 只有改 GPU path 时深入 |
| [../src/graphicsImpl.cpp](../src/graphicsImpl.cpp) | 低到中 | GUI/debug 可视化 |
| [../src/mbff.cpp](../src/mbff.cpp) | 专题 | `cluster_flops` 专题路径 |
| [../test/CMakeLists.txt](../test/CMakeLists.txt) | 高 | CMake regression 注册 |
| [../test/BUILD](../test/BUILD) | 高 | Bazel regression 注册 |
| [../messages.txt](../messages.txt) | 高 | GPL 日志消息清单 |

## 复杂度热点

优先谨慎处理以下区域：

- `nesterovBase.cpp`：文件大，状态多，涉及 filler、bin overlap、density field、GPU sync、callback 动态增删实例。
- `nesterovPlace.cpp`：主循环中 timing、routability、divergence、graphics、reporting 相互交织，改控制流容易造成 QoR 或 runtime 回归。
- `placerBase.cpp`：OpenDB status、row/site、macro/dummy/fixed 分类错误会污染后续所有阶段。
- `routeBase.cpp`：routability inflation 会改变 cell density size 和 target density，容易引入不可逆状态。
- `timingBase.cpp` 和 `clockBase.cpp`：直接影响 STA/resizer 状态，必须注意 virtual/non-virtual repair 和最终 cleanup。
- `src/gpu/` 和 backend factories：CPU/GPU path 必须保持语义一致，尤其是 host/device 坐标和 net box mirror。
- `messages.txt`：新增、删除或修改 `utl::info/warn/error` 消息时要同步消息清单和 doc check。

## 修改代码时的检查清单

提交前至少检查：

- 是否真的改在 GPL 模块内，而不是绕到 `src/sta/`。修改 `src/sta/` 前必须先征求维护者意见。
- 如果新增命令参数，是否同步更新 Tcl、SWIG、`PlaceOptions`、validation、README 和测试。
- 如果新增或修改日志消息，是否同步 `messages.txt`。
- 如果新增回归测试，是否同时更新 `test/CMakeLists.txt` 和 `test/BUILD`。
- 如果改 C++，是否运行 `clang-format -i <files>`。不要格式化 `*.i` 文件。
- 如果改 GPU backend，是否至少覆盖 CPU path 和 GPU path 的差异风险；GPU-only 测试一般是 CMake 条件注册。
- 如果修输出异常，是否追到了数据创建点，而不是只在输出端过滤。
- 是否保留 deterministic 行为，避免无控制的随机数、线程顺序依赖和平台相关输出。

## 推荐测试矩阵

按改动范围选择测试，不必每次全跑。

| 改动范围 | 建议测试 |
| --- | --- |
| 命令参数、普通 placement | `ctest -R '^gpl\.(simple01|simple02|simple03)\.tcl$' --output-on-failure`，`bazel test //src/gpl/test:simple01` |
| initial placement | `simple05`、`simple06`、`medium01`，必要时加一个小型 deterministic regression |
| density/overflow/convergence | `density01`、`convergence01`、`diverge01`、`nograd01` |
| incremental | `incremental01`、`incremental02` |
| timing-driven | `simple01-td`、`simple01-td-tune`、`simple01-td-net-percentage`、`simple01-td-repair` |
| virtual CTS | `simple01-vcts`、`simple01-vcts-clamp`、`simple01-vcts-noclk`、`simple01-vcts-userlat` |
| routability-driven | `simple01-rd`、`simple02-rd`、`simple04-rd` |
| region/multi-domain | `region01`，GPU build 下再看 `region01_gpu`、`region01_gpu_asym` |
| MBFF | `mbff_hier`、`mbff_orig_name`、`mbff_test` |
| FFT/backend | `gpl_fft_unittest`，GPU build 下看 `fft_gpu_test`、`wl_gpu_test` |
| docs/messages | `gpl_readme_msgs_check`、`gpl_messages_txt_check`、`gpl_man_tcl_check` |

## 调试建议

- 从最小 Tcl testcase 开始，不要直接在大型 design 上猜。
- 对 QoR 问题同时记录 HPWL、overflow、target density、bin count、iteration count。
- 对 density 问题先确认 row/site、fixed area、dummy area、filler area 是否合理。
- 对 timing-driven 问题确认 slack 是否存在、net reweight 是否触发、resizer change 是否 virtual。
- 对 routability 问题确认使用的是 RUDY 还是 GRT，并记录 weighted RC、overflowed tiles、inflation area。
- 对 GPU 问题先强制 CPU path 建立参考，再比较 GPU path 的 HPWL、overflow 和最终坐标误差。
- 使用 `global_placement_debug` 定位局部异常，尤其是特定 instance、bin density 或 routability iteration。

## 常见任务路线

### 新增 `global_placement` 参数

1. 在 `PlaceOptions` 中加字段和默认值。
2. 在 `replace.tcl` 中加入 `define_cmd_args` 和 `parse_key_args`。
3. 在 `replace.i::getOptions()` 中解析 key 或 flag。
4. 在 `PlaceOptions::validate()` 中加范围检查。
5. 在实际消费参数的类中使用该字段。
6. 更新 README 参数说明。
7. 添加 Tcl/Python regression，并注册 CMake 和 Bazel。

### 修 placement crash

1. 用最小 testcase 复现。
2. 看 `messages.txt` 中对应 GPL 编号，定位源码。
3. 判断 crash 发生在 PB 初始化、NB 初始化、主循环、timing/routability hook、DB callback 还是 updateDb。
4. 追到最早产生错误状态的位置。
5. 加防御性校验时要明确物理含义，避免把非法 design state 静默吞掉。
6. 加 regression 覆盖原始失败条件。

### 修 QoR 回归

1. 固定线程数、随机种子、输入 design 和命令参数。
2. 同时比较 HPWL、overflow、iteration count、density penalty、wirelength coefficient。
3. 确认变化来自 initial placement、Nesterov core、timing-driven、routability-driven 还是 backend。
4. 对 multi-region 和 macro-heavy design 做额外检查。
5. 如果变动是预期的，需要在 PR 说明中解释 tradeoff 和测试覆盖。

## 术语速查

- HPWL：Half-Perimeter Wirelength，线长估计指标。
- Overflow：超过 bin target density 的面积比例，是 placement 收敛核心指标之一。
- Target density：目标放置密度，越高越拥挤，越低越保守。
- Filler：虚拟 cell，用于 density force 建模和填充 whitespace。
- PB：`PlacerBase` 层，接近 OpenDB 的 placement model。
- NB：`NesterovBase` 层，Nesterov 算法使用的数据 model。
- GCell/GPin/GNet：Nesterov 层的 cell、pin、net。
- RUDY：快速 routing congestion 估计。
- GRT：OpenROAD global router，可提供更真实但更慢的 routability 信息。
- Virtual CTS：placement 期间构造轻量 clock tree 估计 clock latency，结束前清理。
- DB callback：placement 期间 resizer 创建、删除、移动、resize 实例时，同步更新 GPL 内部状态的机制。

## 最小贡献标准

一次可合入的 GPL 改动通常应包含：

- 清楚的 root cause 或需求说明。
- 小而集中的实现，符合现有类边界。
- 覆盖失败点或新行为的 regression。
- CMake 和 Bazel 双注册。
- 必要的 README 或 messages 更新。
- C++ 文件已 clang-format。
- commit 使用 `git commit -s`，满足 DCO。
