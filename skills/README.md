# Skills

本目录存放可被 Agent 调用的技能。

## `aerial-payload-handover`

**空中载荷交接仿真技能**：给定载荷与机型规格，跑仿真、选末端、查释放证书、出报告。
遵循 `SKILL.md`（渐进式披露）：Agent 先读元数据，任务相关时再加载 `references/`。

```
aerial-payload-handover/
├── SKILL.md            # 技能定义（何时用/工作流/脚本/物理速查/坑）
├── references/         # 领域知识（按需加载）
├── scripts/            # 薄封装，调用仓库现有 CLI
└── examples/           # 使用示例
```

## 如何部署到不同 Agent 平台（对应四种实现方式）

| 平台类型 | 做法 |
|---|---|
| **SKILL.md 型**（Claude / Spring AI Alibaba / 智谱等） | 把 `skills/aerial-payload-handover/` 放进平台的技能目录 |
| **Function Calling 型**（OpenAI Agents SDK / LangChain） | 把 `scripts/*.sh` 封装成工具函数注册（或直接调用 `tools/*.py`） |
| **MCP 型**（Claude Desktop / Cursor / Cline） | 写一个 MCP Server 暴露这些脚本为工具；SKILL.md 作使用方法说明 |
| **框架 Skill 系统**（AgentScope Toolkit 等） | 作为"技能"注册，工具组按需激活 |

**推荐组合**：MCP 暴露 `tools/*.py` 能力 + 本 `SKILL.md` 提供使用方法。

## 前置

- 仓库根：`$DRONE_PAYLOAD_CATCH`（默认 `~/drone_payload_catch`）。
- 离线脚本纯 Python；SITL 脚本需 `source env.sh`。
- 技能脚本**无外部 API、无密钥**；副作用仅写入 `report/`（报告/图表）。若不需要写文件，
  可用 `--out /tmp/...` 重定向。

## 安全说明

- 脚本只读配置、只写 `report/`；不删除数据。
- 无网络调用、无凭据。
- `run_sitl_tray.sh` 会启动 Gazebo/PX4 仿真（重资源），运行前会清理同名进程。
