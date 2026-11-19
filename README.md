# 飞特 HD-1910-C001 单颗台架工具

用本地 Agent 或 Linux 命令行完成 **读状态 → 改 ID → 确认 1 Mbps → 回中到已有 2048 → ±50 counts 点动 → 双路断电后的只读复验**。这一路径已在多颗 C001 裸机台架跑通；仅实测 **FEETECH HD-1910-C001，固件 3.46 / mode 4 / phase 52 / resolution 1**。其他型号和固件、返修件、带载或整机均不在验证范围。回中不是编码器校零，本工具也不更改波特率。

**现场风险**：只接正常的单颗未装机无负载舵机。断电下按转接板和舵机标识确认线序、极性及供电，固定外壳，轴周围留空。首次写目标就可能上力，旁护者必须能立即切断外部电源（已用台架为 7.5 V）。异响、卡住、异常发热或失联，立即切外部电源、确认失电；只拔 USB 不够，不抓轴、不解卡。换件/改接线前断开外部电源和 USB。软件不保证异常时卸力。

## 安装一次

1. 将可信发布版本解压或浅克隆到本地独立目录。把包含 `servo_toolkit/`、`C001-RUNTIME.sha256` 和 `skills/` 的目录设为项目根。本工具用 **Linux + Python 3.12**（已验证版本）、`pyserial`，以及 Linux 常见的 `readlink`、`sha256sum`、`fuser`（psmisc）。另需接在该主机的兼容 USB 转 TTL 半双工舵机转接板和合适的外部电源；已跑通的台架使用微雪总线舵机转接板。仅云端对话或没有设备访问权的 Agent 不能代跑实机。
2. 安装前按 [SKILL.md 的清单核对](skills/c001-servo/SKILL.md#开始前) 执行其中两条校验命令，不做串口检查。已有 Python 3.12 + pyserial 3.5 可复用，直接用该解释器跑下方最后两条检查；否则在工具根目录用独立环境安装，不装系统包。按实际路径指定 Python 3.12（下例为 `python3.12`）。

   ```sh
   python3.12 -m venv .venv
   .venv/bin/python -m pip install -r servo_toolkit/requirements.txt
   .venv/bin/python -B -c 'import serial; print(serial.__version__)'
   .venv/bin/python -B -m servo_toolkit --help
   ```

   这两条检查不打开串口。

3. 把 [skills/c001-servo](skills/c001-servo/) **整个目录**放进所用 Agent 的技能目录。常见项目级位置：Codex `.agents/skills/`、Claude Code `.claude/skills/`、Nanobot 当前 Agent 工作区的 `skills/`。按实际版本确认加载，避免覆盖同名技能；没有技能加载器的本地 Agent 可在任务中显式读取 `skills/c001-servo/SKILL.md`。在工具根目录开启 Agent 会话，使用已验证的 Python；也可把根目录和解释器的绝对路径告知 Agent。不要在 `servo_toolkit/` 内建 venv 或夹带其他文件。

### 复制给 Agent 一次安装

复制下面这段，并在消息末尾附上 `来源：<可信本地目录、压缩包路径或固定版本链接>`。

> 请安装 `c001-servo` 和配套 `servo_toolkit`。使用我在消息里提供的可信本地目录、压缩包或指定的 GitHub 版本；没提供就问我，不猜地址。按包根目录的 README 核对清单和文件，安装整个 `skills/c001-servo` 到你的技能目录，已有同名技能先问我。可复用 Python 3.12 和 pyserial；我只允许在独立 venv 中安装 `servo_toolkit/requirements.txt` 所需依赖。用所选解释器做 `import serial` 和 `-B -m servo_toolkit --help` 离线检查，报告工具根目录、解释器、技能位置，并简述用法和风险。不要用 sudo、改系统配置或权限、打开串口或做任何实机操作。

## 开始调试

用户接好单颗、开放串口权限后，可以发：

> 只接了一颗正常、未装机无负载的 HD-1910-C001，串口权限已开放。供电和线序已核对、外壳已固定、轴周围留空，我在现场旁护且能立即切外部电源。请把 ID 从 1 改为 24，现在直接做到回中和 ±50 点动完成。

把 ID 换成本轮实物的值。Agent 按 [SKILL.md](skills/c001-servo/SKILL.md) 快速执行，不反复翻文档或让用户逐条复制命令。手动使用 CLI 也按该页逐阶段核对，不整段粘贴。点动结束后由你断外部电源和 USB、确认失电后重连，再说「两路已断电重连，请只读复验」。串口没权限、保护停止或复验缺项时看 [简短排查](skills/c001-servo/references/execution-entry.md)。符合条件的回中保护停止，断电重连并重新授权后只试一次；再次失败即停止。没当轮现场授权不动作。

## 来源与发布边界

- 动作实现、测试和依赖原样保留。飞特 [FTServo_Python](https://github.com/ftservo/FTServo_Python) SDK 固定在 `9e6e342f6011dc727ba7144fc835bca5357b3ca9`，其许可见 [vendor/LICENSE.ftservo](servo_toolkit/vendor/LICENSE.ftservo)，不用 PyPI 的其他 SDK 替换。
- 分享时只带文档、校验清单、技能和工具源码；不要上传 `records/`（含计划和本地端口信息）、虚拟环境、缓存或其他项目资料。正式开源前由权利人确认本工具与文档的许可证；SDK 自带许可证不等于整个项目已授权开源。
