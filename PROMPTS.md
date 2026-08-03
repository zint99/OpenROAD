## 模块依赖分析

整理一个中文文档，梳理 {{module}} 模块对 odb 哪些 class 有 dependency.

- 强调重点接口与用途，弱化对次要依赖的描述。比如我不关心 {{module}} 模块是否调用了某个 odb class 的 getName() 方法。
- 按照 odb class 对{{module}} 模块的重要程度依次排序。