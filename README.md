# paper-reading

个人论文阅读笔记库。

## 分类规则

**一级分类轴：研究主题。** 每篇文献按其核心研究问题归入对应主题文件夹，而非按架构名称、发表会议或内容形式归类。架构（如 Transformer）与会议（如 ICML/NeurIPS）只是论文的属性，不作为文件夹命名依据。

### 文件夹说明

| 文件夹 | 收录范围 |
|--------|----------|
| `Foundations/` | 深度学习与架构奠基性工作（理论、综述、seminal 架构） |
| `Vision/` | 计算机视觉模型（检测、分割、视觉表征等） |
| `Vision/Generative/` | 图像生成与扩散模型 |
| `Language-Models/` | 语言模型与高效 Transformer |
| `EmbodiedAI/` | 具身智能、机器人学习 |
| `EmbodiedAI/Robotics/` | 机器人操作与控制（绳缆操控、运动规划等） |
| `AI-Infra/` | AI 系统与基础设施（推理优化、KV cache、分布式、Agent 工程等） |
| `Blog/` | 技术博客与方法论文章 |
| `assets/` | 论文附属资源（图片、图表等静态资源，按论文分目录存放） |

### 新文献归类决策树

1. 它是博客/方法论文章吗？ → `Blog/`
2. 它研究深度学习理论或 seminal 架构本身吗？ → `Foundations/`
3. 它处理视觉数据吗？
   - 涉及图像生成/扩散 → `Vision/Generative/`
   - 其他视觉任务（检测/分割/表征） → `Vision/`
4. 它是语言/序列模型吗？ → `Language-Models/`
5. 它涉及具身/机器人吗？
   - 涉及机器人操作与控制（绳缆操控、运动规划等） → `EmbodiedAI/Robotics/`
   - 其他具身智能（导航、交互、具身推理等） → `EmbodiedAI/`
6. 它是 AI 系统工程（推理、训练 infra、Agent 框架）吗？ → `AI-Infra/`
7. 它是论文附属资源（图片、图表等）吗？ → `assets/{论文名}/`

### 命名约定

- 论文优先使用 arXiv ID（如 `2006.16236v3.pdf`），便于追溯。
- 无 arXiv ID 的按「主题关键词」命名。
- 会议/架构信息可作为文件名前缀或标签，但不决定文件夹归属。

### 重构记录

2026-08-05：新增 `assets/` 目录与 `EmbodiedAI/Robotics/` 子目录。
- `assets/`：集中存放论文附属资源（图片、图表等），按论文名分目录组织，保持论文文件夹整洁。
- `EmbodiedAI/Robotics/`：从 `EmbodiedAI/` 中拆分出机器人操作与控制子类，细化具身智能分类粒度。

2026-08-04：将原混合分类（主题/架构/会议/内容类型四种标准并存）统一为「研究主题」单一轴线。
- `DeepLearning/` → `Foundations/`（重命名，并纳入 seminal 架构论文）
- `ICML/` → `AI-Infra/`（按主题而非会议归类，可容纳 NeurIPS/OSDI 等同类论文）
- `Transformer/` → 拆分为 `Vision/` 与 `Language-Models/`（视觉与语言两线分家）
- 新增 `Vision/Generative/`（补齐图像生成模型分类缺口）
