# `servo_toolkit`：飞特 HD-1910-C001 台架模块

从本目录的**上一级**运行 `python3 -B -m servo_toolkit --help`。安装、支持型号、现场风险与可复制的 Agent 安装提示词见 [工具根目录 README](../README.md)；逐颗执行只用 [c001-servo](../skills/c001-servo/SKILL.md)。

- `__main__.py` 是 CLI；`motion.py` 提供 `motion-prepare` / `motion-run` 一次性受控入口；`servo.py` 负责单颗读写和动作；`strict_sdk.py` 封装串口事务。
- `requirements.txt` 只声明 pyserial；`vendor/` 保留固定版本飞特 SDK 与其 [许可证](vendor/LICENSE.ftservo)，`tests/` 是原有离线测试。实现、依赖、测试及默认关闭的旧动作门禁均未修改。
- `records/` 是工具生成的本机记录与动作计划，不随发布包分发。CLI 仍可能显示旧候选提示；当前适用范围以实测的 C001 配置和上述文档为准，不因此扩大支持型号。
