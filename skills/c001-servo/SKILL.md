---
name: c001-servo
description: '飞特 FEETECH HD-1910-C001 单颗桌面调试：检查串口、改 ID、确认 1 Mbps、回到已有中位 2048、±50 counts 点动、断电重连只读复验。用于 Linux 上已接好的单颗无负载舵机，不用于其他型号或整机。'
---

# C001 单颗台架快速流程

**正常一颗只走这一页。** 不逐颗翻教程、源码、历史日志，不跑离线测试，不另写报告，只留工具自动生成的 `servo_toolkit/records/`。只要求检查或改号就在相应阶段结束；只要求重连复验就走第 4 步，不改号、不动作。已有本轮信息不重复询问。安装或遇到问题才查其他说明。安装完成时只提示用法和风险，不连串口、不动作。

## 开始前

- 只用于**飞特 HD-1910-C001** 已实测配置（读回 `7946/10/31`、固件 `3.46`、`mode=4, phase=52, resolution=1`）。其他型号、固件、疑似异常或返修件未验证，不能套用。由用户确认仅接一颗未装机、无负载的舵机，线序/极性与供电正确、外壳固定、轴周围留空、手不靠近，且本人现场能立即切断外部电源（原台架为 7.5 V）。这些不是程序可证明的条件。
- 本轮确认接好、已开放权限及当前 ID 来源；要改号才需用户指定目标 ID（0–253）。新出厂件可用 `1` 作**一次只读候选**；失败不扫描、不猜写。用已安装的工具根目录和解释器，优先沿用已确认的绝对串口路径（通常是 `/dev/serial/by-id/…`）；首次或设备不明确时只核对设备路径，不开多个串口进程。
- 每次接入/重连快速校验：在工具根目录运行以下检查；`PORT` 已设为经确认的串口路径。缺权限由用户开放当前设备节点（重连后可能要重开）；不代 `sudo`，不改系统配置，不杀占用进程。

```sh
set -eu
printf '%s  %s\n' eaa8523b6b166bd536bc5926c9961b5e39d167899ec1161a9aa925a05b5c61c2 C001-RUNTIME.sha256 | sha256sum -c --strict --quiet
sha256sum -c --strict --quiet C001-RUNTIME.sha256
NODE=$(readlink -e -- "$PORT") || { echo '串口不存在'; exit 1; }
[ -c "$NODE" ] && [ -r "$NODE" ] && [ -w "$NODE" ] || { echo '串口不是字符设备或读写权限不足'; exit 1; }
if usage=$(fuser "$NODE" 2>&1); then
  printf '串口被占用：%s\n' "$usage"; exit 1
else
  rc=$?; [ "$rc" -eq 1 ] && [ -z "$usage" ] || { printf '占用检查失败：%s\n' "$usage"; exit 1; }
fi
```

`fuser` 未发现占用不能替代人工关闭其他串口程序；校验失败或工具不可用就停，不用别的版本或环境绕过。

## 直接执行

从工具根目录运行。`PYTHON` 是已验证的解释器，`CURRENT_ID` 是本轮读取的当前 ID，`ID` 是用户指定的目标 ID（不改号就用当前 ID）。`run_c001` 是 CLI 简写。**用实际值逐阶段执行，核对通过才继续，不整块复制跑。**

```sh
run_c001() { "$PYTHON" -B -m servo_toolkit --port "$PORT" --baud 1000000 "$@"; }
run_c001 inspect --id "$CURRENT_ID"
run_c001 set-id --id "$CURRENT_ID" --new-id "$ID"  # 同 ID 或已确认改好就跳过
run_c001 motion-prepare --id "$ID" --scope "$SCOPE"
run_c001 motion-run --id "$ID" --scope "$SCOPE" --plan "$PLAN_ID" --approve "$APPROVE_SHA256"
```

1. **先读再写**：`inspect` 退出 0，且型号/固件/模式命中上方范围，`endian=0, baud_index=0, response_level=1, lock=1`、`torque/speed/moving/status` 均为 0、电压读回在 4–8.4 V、当前位置和 2048 在有效单圈限位内才继续。按用户指定的目标改号，确认退出 0、`snapshot.id` 为新 ID。1 Mbps 可通信且 `baud_index=0` 就无需改波特率；`--baud` 只设电脑端速率，`set-baud` 未实现。改号报错时写入可能已生效，停下只读排查，不重写。
2. **回中/点动按本轮授权**：只给 ID 或开权限不是动作授权。用户当轮明确授权所需动作且确认现场旁护时，一句话报范围即可继续，不在 prepare 后重复问；否则先说明动作范围，等本轮现场授权。仅回中用 `SCOPE=center`，两项用 `SCOPE=center-jog50`。`motion-prepare` 只读设备；确认退出 0、`state=prepared_read_only`，取本次 `plan_id` 和 `approve_sha256`。同端口、ID、范围执行 `motion-run` **一次**，回中通过才点动。计划 300 秒有效且只能用一次，摘要不代替授权；过期、失败、重连、换件、叫停或失去旁护，不续跑、不复用。用能等待进程正常退出的执行方式，不用短硬超时强杀（180 秒运动预算和 300 秒计划寿命不是进程硬截止）。回中是到已有 2048，不校零；不调用旧 `jog`/`move-center`、`calibrate-center`/`0x0B`，不改门禁或用裸 SDK 绕过。
3. **报告结果**：退出 0、`state=passed`，所请求阶段与 `final_inspect` 均 `passed` 才通过。最终位置须在 2045–2051 counts，已卸力、静止、状态为 0、配置未变。点动固定 ±50，回原位偏差 ≤12 counts；已在中位窗的回中可能只读未动。简报 ID、1 Mbps、实际位置和结果；点动通过但未重连时标注掉电复验待做。
4. **双路断电重连后只读复验**：用户断外部电源和 USB、确认失电再重连。重新做快速检查，再 `run_c001 inspect --id "$ID"`；与动作后的 `final_inspect.snapshot`（仅改号时用 `set-id.snapshot`）比对身份与配置，确认 `baud_index=0, lock=1, torque/speed/moving/status=0`，做过回中才验 2045–2051。改过号则另开连接 `inspect --id "$OLD_ID"`（保留原始旧 ID，重试不覆盖），只把旧 ID `PING` 的 `-6 / 0x00` 判为预期无应答；同 ID 跳过。`goal_raw` 是否为 0 不作门槛。未确认双路断电或缺少基线时如实报告；位置偏离不自动回中。
   - 比对字段：`id, ping_model_number, firmware_major, firmware_minor, model_low, model_high, endian, baud_index, secondary_id, response_level, angle_min, angle_max, phase, resolution, offset_raw, mode, lock`。

## 异常：停下，不自动重试

动作异常、失联或需中止正在进行的运动时，先请用户**立即切外部电源、确认失电**，不是只拔 USB；不抓轴、不通电解卡。未确认卸力就报「出力状态未知」；软件不能保证失联或卡死时卸力。保护停止不等于故障已确诊。

**只允许一次人工重试**：`stage=center`、`center.status=failed_or_uncertain`、`jog50.status=not_run`，且明确报「步间尚未静止」并已回读扭矩关闭，才提示可在双路断电、确认失电后重新连接。用户确认同一颗、无卡住/异响/异常发热，且重新授权并旁护后，重查端口和权限，只读核对目标 ID、静止卸力及与失败前一致的身份/配置（位置可以不同），不再改号。新建计划，按新授权范围执行一次 `motion-run`；重连或换计划不重置次数。重试再失败、先前次数不明或不是此类保护停，停止本颗动作，保留记录供人工排查，不再尝试或放宽保护。

首次使用可提示：**把单颗 C001 接好、开串口权限，告诉我当前/目标 ID 和是否允许现场回中、点动；异常立即切外部电源。** 权限、复验字段及错误判读见 [按需排查](references/execution-entry.md)，正常运行不必打开。
