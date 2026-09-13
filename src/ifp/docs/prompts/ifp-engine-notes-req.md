# ifp-engine-notes-req

面向新员工写一篇关于 `ifp` 模块的中文技术笔记，满足以下需求：

- 你的目标是帮助新员工快速理解 `ifp` 模块的总体逻辑，而非死扣细节。
- 淡化 UI 层面的繁琐细节，如有需要，只需讲解 `.tcl` 和 `.i` 中的必要逻辑。
- 只需要向新员工讲解 `initialize_floorplan` 的基础内容，包括: 1.specify manually die/core area; 2.specify the utilization/aspect ratio; 不需要介绍 hybrid row 等 advance topic.
- 讲解 `make_rows` 和 `make_tracks` 命令中的必要逻辑。
- 如果有需要，可以在文档中配图。
- 分别给1.OpenROAD 工具使用者；2.OpenROAD 其他模块开发者；3.ifp 模块开发者提出该模块的后续学习建议。