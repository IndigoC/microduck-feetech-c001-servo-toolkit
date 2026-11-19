# FEETECH HD-1910-C001 Single-Servo Bench Toolkit

**English** · [简体中文](README.zh-CN.md)

Check and prepare one servo at a time before installing it in MicroDUCK.

Use a local agent or the Linux command line to **read status → change ID → verify communication at 1 Mbps → return to the existing center at 2048 counts → jog by ±50 counts → perform a read-only recheck after both external power and USB are disconnected and reconnected**.

- **Tested:** Multiple **FEETECH HD-1910-C001** servos, one at a time, unmounted and unloaded, with firmware `3.46`, `mode=4`, `phase=52`, and `resolution=1`.
- Other models and firmware, loaded setups, and assembled robots are outside the tested scope.

## ⚠️ Bench safety

- **Connect and secure:** Connect only one known-good, unmounted, unloaded servo. With power disconnected, check the pinout, polarity, and supply against the adapter and servo labels. Secure the housing and keep the area around the shaft clear.
- **Before motion:** Make sure the servo can move normally before first use. Sending a target position can apply torque. The person supervising at the bench must be able to cut external power immediately.
- **If something goes wrong:** If you notice unusual noise, binding, overheating, or loss of communication, immediately cut external power, disconnect USB, and confirm the servo is de-energized.
- **Before swapping or rewiring:** Disconnect both external power and USB.

## 🛠️ Setup

- **Software:** Tested with Linux, Python 3.12, and pyserial 3.5. You also need `readlink`, `sha256sum`, and `fuser` (from `psmisc`).
- **Hardware:** A compatible USB-to-TTL half-duplex bus servo adapter and a suitable external power supply. The tested bench used a Waveshare bus servo adapter.
- **Device access:** A cloud-only chat or an agent without serial device access cannot operate the hardware.

### 1. Get and verify the toolkit

- Extract a trusted release archive or shallow-clone a trusted revision into a dedicated local directory.
- Run the commands below from the toolkit root, which contains `servo_toolkit/`, `C001-RUNTIME.sha256`, and `skills/`.
- Verify the checksum manifest first, then the runtime files it lists. Both checks must pass before installation. Neither accesses the serial port.

```sh
printf '%s  %s\n' \
  eaa8523b6b166bd536bc5926c9961b5e39d167899ec1161a9aa925a05b5c61c2 C001-RUNTIME.sha256 \
  | sha256sum -c --strict --quiet &&
sha256sum -c --strict --quiet C001-RUNTIME.sha256
```

### 2. Prepare the Python environment

- **Already have Python 3.12 and pyserial 3.5?** Skip the first two commands. Use your interpreter instead of `.venv/bin/python` in the last two.
- **Need a new environment?** Create `.venv` at the toolkit root, not inside `servo_toolkit/`. Do not add other files to `servo_toolkit/` or install system packages. Use the actual path to your Python 3.12 interpreter if `python3.12` is not on your `PATH`.

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r servo_toolkit/requirements.txt
.venv/bin/python -B -c 'import serial; print(serial.__version__)'
.venv/bin/python -B -m servo_toolkit --help
```

The last two checks do not open a serial port.

### 3. Install the agent skill (optional)

- Copy the **entire** [`skills/c001-servo/`](skills/c001-servo/) directory into your agent's skills directory.
- Check that your agent version loads the skill. Do not overwrite a skill with the same name without approval. If your local agent has no skill loader, ask it to read [`SKILL.md`](skills/c001-servo/SKILL.md) directly.
- Start the agent session from the toolkit root with the verified Python interpreter, or give the agent the absolute paths to both.

### Copyable agent setup prompt

Add `Source: <trusted local directory, archive path, or pinned release link>` to the end of this prompt:

> Please install the `c001-servo` skill and its companion `servo_toolkit`.
>
> - **Source:** Use the trusted local directory, archive, or specific GitHub revision I provide. If I haven't provided a source, ask me rather than guessing.
> - **Verify and install:** Follow the root README to verify the manifest and runtime files. Install the entire `skills/c001-servo/` directory into your skills directory. Ask before replacing a skill with the same name.
> - **Python environment:** You may reuse Python 3.12 with pyserial 3.5. If you need dependencies, install only those in `servo_toolkit/requirements.txt`, and only in a dedicated virtual environment.
> - **Check and report:** Use the selected interpreter to check `import serial` and `-B -m servo_toolkit --help` without opening a serial port. Report the toolkit root, interpreter path, and skill location, and briefly explain usage and risks.
> - **Setup limits:** Do not use `sudo`, change system settings or permissions, open a serial port, or operate the hardware during setup.

## 🎛️ Run a bench session

After connecting one servo and granting serial port access, you can give your local agent a request like this:

> Only one known-good, unmounted, unloaded HD-1910-C001 is connected, and the serial port is accessible. The wiring, polarity, and supply have been checked, the housing is secure, and the area around the shaft is clear. Someone is supervising on site and can cut external power immediately. Change the ID from 1 to 24, then proceed now through centering and the ±50-count jog test.

- **Run:** Replace the example IDs with the values for your servo. Follow the [bench procedure](skills/c001-servo/SKILL.md) one stage at a time, whether you use an agent or the CLI. Do not move the servo without explicit authorization for this session and on-site supervision.
- **Recheck:** After jogging, disconnect external power and USB, confirm that power is off, then reconnect both. Tell the agent: “I disconnected external power and USB, confirmed the servo was de-energized, and reconnected both. Please perform a read-only recheck.”
- **Troubleshoot:** For serial access problems, protective stops, or incomplete rechecks, see the [troubleshooting reference](skills/c001-servo/references/execution-entry.md). Only the specific centering stop described in the bench procedure permits one retry, after a full disconnect and reconnect and renewed on-site authorization. If that retry fails, stop.

The bench procedure and troubleshooting reference are in Chinese.

## 📄 Code sources and licensing

- **Bundled SDK:** The [pinned FEETECH FTServo_Python SDK](https://github.com/ftservo/FTServo_Python/tree/9e6e342f6011dc727ba7144fc835bca5357b3ca9) uses the MIT License. Keep the original copyright notice and license text when distributing the SDK; see [`LICENSE.ftservo`](servo_toolkit/vendor/LICENSE.ftservo).
- **This toolkit:** The project's original toolkit code, tests, documentation, and agent skill are licensed under the [MIT License](LICENSE). Third-party code retains its own license.
