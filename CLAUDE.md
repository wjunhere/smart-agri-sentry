# CLAUDE.md

* 项目上下文请看 .claude\PROJECT\_CONTEXT .md，并且要求更新上下文的时候，一定要按照PROJECT\_CONTEXT .md要求渐进式披露，不同的内容放到/docs下不同模块的文档中，当完成一项功能或修复什么问题时要更新上下文

* ros2开发例程放到了example\，RDKX5资料放到了docs\hardware\_refs\RDK\_X5，STM32F103RCT6资料放到了docs\hardware\_refs\stm32f103rct6，STM32F407ZGT6的资料放到了docs\hardware\_refs\stm32f407zgt6，Lora模块的资料放到了docs\hardware\_refs\lora\_E22\_400TBH\_SC，传感器资料放到docs\sensors,资料我都转化为md文档,HAL库我放到了E:\stm32cubeMXrepository

* 项目的github仓库是wjunhere/smart-agri-sentry

* 如果要完成某个复杂的任务时，需要先开一个分支，按照 Superpowers 完整流程，先 brainstorm，再 write-plan，最后 execute-plan，必须做 TDD 和 code review

* 可以通过ssh连接到RDKX5板端 ，板端环境是Ubuntu22.04，ssh rdk ，密码是sunrise，pc端设置为无密登录，项目放在了\~\dev\_ws,项目从本项目克隆而来，只是文件夹名重命名为dev\_ws， 通过git进行版本管理

* 上位机RDK代码优先先在本地进行开发，本地没有ros2运行的环境，先提交到远程仓库，再在RDK板端拉取仓库，进行测试

* 对于STM32开发，系统已经安装好了STM3CubeMX，STM32CubeCLT,STM32\_Programmer\_CLI等cli工具,优先GCC编译stm32文件

* 本地已安装了wsl2，系统为ubuntu22.04，密码是wjun

## Windows 提权 (gsudo)

当需要管理员权限执行命令时（如创建软链接、修改系统配置、写机器级环境变量），使用 gsudo 提权：

* gsudo 本体路径：`C:\Program Files\gsudo\2.6.1\gsudo.exe`（已加入用户级 PATH，新开的终端可直接用 `gsudo`；Git Bash 旧会话需用全路径）
* Git Bash 中调用简单命令：`"/c/Program Files/gsudo/2.6.1/gsudo.exe" powershell -NoProfile -Command "<命令>"`
* 复杂命令（多层引号/变量）不要内联，先写成临时 `.ps1` 脚本再执行，用完删除：`gsudo powershell -NoProfile -ExecutionPolicy Bypass -File <脚本.ps1>`（bash→gsudo→PowerShell 三层转义容易出错，此方式最稳）
* PowerShell 脚本里调用系统程序时用全路径（如 `$env:SystemRoot\System32\whoami.exe`），避免命中 Git Bash 的 `/usr/bin` 同名 GNU 程序
* 提权时用户桌面可能弹 UAC 确认框，需用户点"是"才能继续；未被批准时命令会失败，不要反复重试

## 板端进程管理（必须遵守）

* **栈内节点只能通过 `scripts/rdk/start_robot_stack.sh` / `stop_robot_stack.sh`（或前端按钮）启停**，不要在 SSH 里用 `ros2 run` 单独拉起相机等栈内节点后不管——stop 脚本按进程名模式清理，手动拉起的进程会成为孤儿：占住 USB 相机设备（下次 launch 报 `0x80000203` 崩溃）并持续吃 CPU（Nav2 被饿到行为树超时，车速变慢）。
* **新增任何栈内节点时，必须同步把节点进程名加进 `stop_robot_stack.sh` 的 `stop_patterns` 清理名单**，否则停止栈时该节点必然残留（2026-07-17 海康相机漏加导致巡航变慢的教训）。
* 调试确需单独起节点时，用完立即 kill，并在预热前确认无残留：`ps -eo pid,args | grep <节点名> | grep -v grep`。
* 板端排查"车不动/变慢"类问题时，先查残留进程和节点列表（`ros2 node list` 应无重复节点），再查 `/tmp/sentry_v2_start_robot_stack.log` 里的 `process has died` 和行为树超时告警。

## 任务追踪与状态管理 (PLAN.md)

为了支持跨会话的复杂任务开发，你必须严格维护一个名为 `PLAN.md` 的文件（位于项目根目录）。

**执行规则：**

1. **初始化**：在制定计划阶段，必须创建或重置 `PLAN.md`，将大目标拆解为具体的、可执行的步骤列表（使用 Markdown 复选框 `- [ ]`）。
2. **实时更新**：在编码执行阶段，每完成一个步骤或子任务，**必须立即**更新 `PLAN.md`，将对应的复选框标记为完成 `- [x]`。
3. **状态记录**：如果某个步骤失败或遇到阻塞，请在文件中记录简短的备注。
4. **会话结束**：在生成最终回复前，确保 `PLAN.md` 的状态是最新的。

<!-- CODEGRAPH_START -->

## CodeGraph

This project has a CodeGraph MCP server (`codegraph_*` tools) configured. CodeGraph is a tree-sitter-parsed knowledge graph of every symbol, edge, and file. Reads are sub-millisecond and return structural information grep cannot.

### When to prefer codegraph over native search

Use codegraph for **structural** questions — what calls what, what would break, where is X defined, what is X's signature. Use native grep/read only for **literal text** queries (string contents, comments, log messages) or after you already have a specific file open.

| Question                                                  | Tool                                                                                 |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| "Where is X defined?" / "Find symbol named X"             | `codegraph_search`                                                                   |
| "What calls function Y?"                                  | `codegraph_callers`                                                                  |
| "What does Y call?"                                       | `codegraph_callees`                                                                  |
| "How does X reach/become Y? / trace the flow from X to Y" | `codegraph_trace` (one call = the whole path, incl. callback/React/JSX dynamic hops) |
| "What would break if I changed Z?"                        | `codegraph_impact`                                                                   |
| "Show me Y's signature / source / docstring"              | `codegraph_node`                                                                     |
| "Give me focused context for a task/area"                 | `codegraph_context`                                                                  |
| "See several related symbols' source at once"             | `codegraph_explore`                                                                  |
| "What files exist under path/"                            | `codegraph_files`                                                                    |
| "Is the index healthy?"                                   | `codegraph_status`                                                                   |

### Rules of thumb

* **Answer directly — don't delegate exploration.** For "how does X work" / architecture questions, answer with 2-3 codegraph calls: `codegraph_context` first, then ONE `codegraph_explore` for the source of the symbols it surfaces. For a specific **flow** ("how does X reach Y") start with `codegraph_trace` from→to — one call returns the whole path with dynamic hops bridged — then ONE `codegraph_explore` for the bodies; don't rebuild the path with `codegraph_search` + `codegraph_callers`. Codegraph IS the pre-built index, so spawning a separate file-reading sub-task/agent — or running a grep + read loop — repeats work codegraph already did and costs more for the same answer.

* **Trust codegraph results.** They come from a full AST parse. Do NOT re-verify them with grep — that's slower, less accurate, and wastes context.

* **Don't grep first** when looking up a symbol by name. `codegraph_search` is faster and returns kind + location + signature in one call.

* **Don't chain** **`codegraph_search`** **+** **`codegraph_node`** when you just want context — `codegraph_context` is one call.

* **Don't loop** **`codegraph_node`** **over many symbols** — one `codegraph_explore` call returns several symbols' source grouped in a single capped call, while each separate node/Read call re-reads the whole context and costs far more.

* **Index lag**: the file watcher debounces \~500ms behind writes; don't re-query immediately after editing a file in the same turn.

### If `.codegraph/` doesn't exist

The MCP server returns "not initialized." Ask the user: *"I notice this project doesn't have CodeGraph initialized. Want me to run* *`codegraph init -i`* *to build the index?"*

<!-- CODEGRAPH_END -->
