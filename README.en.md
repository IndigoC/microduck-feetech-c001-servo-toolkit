# FEETECH HD-1910-C001 Single-Servo Bench Toolkit

**English** · [简体中文](README.md)

Use a local agent or the Linux command line to **read the servo status, change its ID, verify communication at 1 Mbps, return it to its existing center at 2048 counts, jog by ±50 counts, and perform a read-only recheck after disconnecting and reconnecting both external power and USB**.

This workflow has been tested on multiple **FEETECH HD-1910-C001** servos, one at a time, unmounted and unloaded, with firmware `3.46`, `mode=4`, `phase=52`, and `resolution=1`. Other models and firmware, repaired units, loaded setups, and assembled robots are outside the tested scope. Centering does not recalibrate the encoder zero, and the toolkit does not change the baud rate.

## Bench safety

Connect only one known-good, unmounted, unloaded servo. With power disconnected, verify the pinout, polarity, and supply against the labels on the adapter and servo. Secure the servo housing and keep hands and objects clear of the shaft. The first target position command can apply torque. The person at the bench must be able to cut external power immediately. The tested bench used a 7.5 V external supply.

If you hear unusual noise or notice binding, overheating, or loss of communication, **cut external power immediately and confirm the servo is de-energized. Unplugging USB alone is not enough.** Do not grab the shaft or try to free a jam. Disconnect both external power and USB before swapping the servo or changing the wiring. Software cannot guarantee torque release if something goes wrong.

## One-time setup

1. Extract a trusted release archive or shallow-clone a trusted revision into a dedicated local directory. Work from the toolkit root, which contains `servo_toolkit/`, `C001-RUNTIME.sha256`, and `skills/`. The tested software setup is **Linux, Python 3.12, and pyserial 3.5**. The workflow also uses `readlink`, `sha256sum`, and `fuser` (from `psmisc`). To operate the servo, the host needs a compatible USB-to-TTL half-duplex bus servo adapter and a suitable external supply. The tested bench used a Waveshare bus servo adapter. A cloud-only chat or an agent without serial device access cannot operate the hardware.

2. From the toolkit root, verify the checksum manifest and the runtime files it lists. Both checks must pass before installation; these commands do not access the serial port.

   ```sh
   printf '%s  %s\n' \
     eaa8523b6b166bd536bc5926c9961b5e39d167899ec1161a9aa925a05b5c61c2 C001-RUNTIME.sha256 \
     | sha256sum -c --strict --quiet &&
   sha256sum -c --strict --quiet C001-RUNTIME.sha256
   ```

3. If you already have Python 3.12 with pyserial 3.5, use that interpreter to run just the last two commands below. Otherwise, use a Python 3.12 installation that can create a virtual environment with `pip`. Set up the environment in the toolkit root, not inside `servo_toolkit/`, and install no system packages. Use the actual path to your Python 3.12 interpreter if `python3.12` is not on your `PATH`.

   ```sh
   python3.12 -m venv .venv
   .venv/bin/python -m pip install -r servo_toolkit/requirements.txt
   .venv/bin/python -B -c 'import serial; print(serial.__version__)'
   .venv/bin/python -B -m servo_toolkit --help
   ```

   The import and CLI help checks do not open a serial port.

4. To use an agent, copy the **entire** [`skills/c001-servo/`](skills/c001-servo/) directory into its skills directory. Typical locations are `.agents/skills/` for Codex, `.claude/skills/` for Claude Code, and `skills/` in the active agent workspace for Nanobot. Verify that your agent version loads the skill, and do not overwrite an existing skill with the same name without approval. If your local agent has no skill loader, explicitly ask it to read [`SKILL.md`](skills/c001-servo/SKILL.md). Start the agent session from the toolkit root with the verified Python interpreter, or provide the agent with both absolute paths.

### Copyable agent setup prompt

Add `Source: <trusted local directory, archive path, or pinned release link>` to the end of this prompt:

> Please install the `c001-servo` skill and its companion `servo_toolkit` from the trusted local directory, archive, or specific GitHub release I provide. If I haven't provided a source, ask me rather than guessing. Follow the root README to verify the manifest and runtime files, then install the entire `skills/c001-servo/` directory into your skills directory. Ask before replacing an existing skill with the same name. You may reuse Python 3.12 with pyserial 3.5. If dependencies are needed, install only those in `servo_toolkit/requirements.txt`, and only in a dedicated virtual environment. Using the selected interpreter, check `import serial` and `-B -m servo_toolkit --help` without opening a serial port. Report the toolkit root, interpreter path, and skill location, and briefly explain usage and risks. Do not use `sudo`, change system settings or permissions, open a serial port, or operate the hardware during setup.

## Run a bench session

After connecting one servo and granting serial port access, you can give your local agent a request like this:

> Only one known-good, unmounted, unloaded HD-1910-C001 is connected, and the serial port is accessible. I've checked the wiring, polarity, and supply, secured the housing, and cleared the area around the shaft. I'm supervising at the bench and can cut external power immediately. Change the ID from 1 to 24, then proceed now through centering and the ±50-count jog test.

Replace the example IDs with the values for your servo. The agent follows the [bench procedure](skills/c001-servo/SKILL.md) without asking you to paste each command. If using the CLI manually, follow the same procedure one stage at a time and verify each result before continuing. Do not run the entire command block at once. Centering and jogging require explicit authorization for the current session while someone is supervising at the bench.

After the jog test, disconnect external power and USB, confirm that power is off, then reconnect them and tell the agent: “I disconnected external power and USB, confirmed power was off, and reconnected both. Please perform a read-only recheck.”

For serial access problems, protective stops, or incomplete rechecks, see the [troubleshooting reference](skills/c001-servo/references/execution-entry.md). The detailed bench procedure and troubleshooting reference are in Chinese. Do not retry automatically. Only the specific centering stop described in the bench procedure permits one retry after a full disconnect and reconnect and renewed on-site authorization. If that retry fails, stop.

## Third-party code and release scope

- The motion implementation, tests, and dependencies are unchanged. The vendored [FEETECH FTServo_Python SDK](https://github.com/ftservo/FTServo_Python) is pinned to commit `9e6e342f6011dc727ba7144fc835bca5357b3ca9`. Its MIT license is in [`servo_toolkit/vendor/LICENSE.ftservo`](servo_toolkit/vendor/LICENSE.ftservo). Do not substitute another SDK from PyPI.
- When packaging the toolkit, include only the documentation, checksum manifest, complete skill directory, and toolkit source. Do not upload `servo_toolkit/records/` (which contains motion plans and local serial port details), virtual environments, caches, or unrelated project material. Before publishing as open source, confirm the license for the toolkit, documentation, and skill with the rights holders. The SDK's license does not cover the entire toolkit.
