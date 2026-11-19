# 飞特 HD-1910-C001 单颗台架工具

**简体中文** · [English](README.md)

用于 MicroDUCK 装机前的单颗舵机检查与准备。

用本地 Agent 或 Linux 命令行完成 **读状态 → 改 ID → 确认 1 Mbps → 回中到已有 2048 → ±50 counts 点动 → 双路断电后的只读复验**。

- **实测范围**：目前已在多颗 C001 裸机台架跑通；仅实测 **FEETECH HD-1910-C001，固件 3.46 / mode 4 / phase 52 / resolution 1**。
- 其他型号和固件、带载或整机均不在验证范围。

## ⚠️ 现场风险

- **接线与固定**：只接正常的单颗未装机无负载舵机。断电下按转接板和舵机标识确认线序、极性及供电，固定外壳，轴周围留空。
- **动作前**：首次使用之前确保舵机可以正常转动，写目标就可能上力，旁护者必须能立即切断外部电源。
- **异常时**：异响、卡住、异常发热或失联，立即切外部电源和USB、确认失电。
- **换件或改接线**：换件/改接线前断开外部电源和 USB。

## 🛠️ 安装说明

- **软件**：已验证 Linux、Python 3.12 和 pyserial 3.5。另需 `readlink`、`sha256sum` 和 `fuser`（`psmisc`）。
- **硬件**：兼容的 USB 转 TTL 半双工舵机转接板和合适的外部电源。已跑通的台架使用微雪总线舵机转接板。
- **实机访问**：仅云端对话或没有设备访问权的 Agent 不能代跑实机。

### 1. 获取工具并校验

- 将可信发布版本解压或浅克隆到本地独立目录。
- 下方命令都在工具根目录运行，即包含 `servo_toolkit/`、`C001-RUNTIME.sha256` 和 `skills/` 的目录。
- 先核对校验清单本身，再核对清单里的运行文件。两项通过后再安装；不做串口检查。

```sh
printf '%s  %s\n' \
  eaa8523b6b166bd536bc5926c9961b5e39d167899ec1161a9aa925a05b5c61c2 C001-RUNTIME.sha256 \
  | sha256sum -c --strict --quiet &&
sha256sum -c --strict --quiet C001-RUNTIME.sha256
```

### 2. 准备 Python 环境

- **已有 Python 3.12 + pyserial 3.5**：跳过前两条命令，用已有解释器替换最后两条中的 `.venv/bin/python`。
- **需要新建环境**：在工具根目录创建 `.venv`，不要在 `servo_toolkit/` 内建 venv 或夹带其他文件。不装系统包；按实际路径指定 Python 3.12（下例为 `python3.12`）。

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r servo_toolkit/requirements.txt
.venv/bin/python -B -c 'import serial; print(serial.__version__)'
.venv/bin/python -B -m servo_toolkit --help
```

最后两条检查不打开串口。

### 3. 安装 Agent 技能（可选）

- 把 [skills/c001-servo](skills/c001-servo/) **整个目录**放进所用 Agent 的技能目录。
- 按实际版本确认加载，避免覆盖同名技能；没有技能加载器的本地 Agent 可在任务中显式读取 [`skills/c001-servo/SKILL.md`](skills/c001-servo/SKILL.md)。
- 在工具根目录开启 Agent 会话，使用已验证的 Python；也可把根目录和解释器的绝对路径告知 Agent。

### 复制给 Agent 一次安装

复制下面这段，并在消息末尾附上 `来源：<可信本地目录、压缩包路径或固定版本链接>`。

> 请安装 `c001-servo` 和配套 `servo_toolkit`。
>
> - **来源**：使用我在消息里提供的可信本地目录、压缩包或指定的 GitHub 版本；没提供就问我，不猜地址。
> - **核对与安装**：按包根目录的 README 核对清单和文件，安装整个 `skills/c001-servo/` 到你的技能目录，已有同名技能先问我。
> - **Python 环境**：可复用 Python 3.12 和 pyserial；我只允许在独立 venv 中安装 `servo_toolkit/requirements.txt` 所需依赖。
> - **检查与反馈**：用所选解释器做 `import serial` 和 `-B -m servo_toolkit --help` 离线检查，报告工具根目录、解释器、技能位置，并简述用法和风险。
> - **安装限制**：不要用 sudo、改系统配置或权限、打开串口或做任何实机操作。

## 🎛️ 开始调试

用户接好单颗、开放串口权限后，可以发：

> 只接了一颗正常、未装机无负载的 HD-1910-C001，串口权限已开放。供电和线序已核对、外壳已固定、轴周围留空，现场旁护且能立即切外部电源。请把 ID 从 1 改为 24，现在直接做到回中和 ±50 点动完成。

- **执行**：把 ID 换成本轮实物的值。Agent 按 [SKILL.md](skills/c001-servo/SKILL.md) 快速执行，手动使用 CLI 也按该页逐阶段核对。没当轮现场授权不动作。
- **复验**：点动结束后断外部电源和 USB、确认失电后重连，再说「两路已断电重连，请只读复验」。
- **排查**：串口没权限、保护停止或复验缺项时看 [简短排查](skills/c001-servo/references/execution-entry.md)。符合条件的回中保护停止，断电重连并重新授权后只试一次；再次失败即停止。

## 📄 代码来源与许可

- **内置 SDK**：[飞特 FTServo_Python（固定版本）](https://github.com/ftservo/FTServo_Python/tree/9e6e342f6011dc727ba7144fc835bca5357b3ca9) 采用 MIT 许可。分发 SDK 时须保留原版权声明和许可文本，见 [`LICENSE.ftservo`](servo_toolkit/vendor/LICENSE.ftservo)。
- **本工具**：本项目原创的工具代码、测试、文档和 Agent 技能采用 [MIT 许可](LICENSE)。第三方代码保留各自的许可。
