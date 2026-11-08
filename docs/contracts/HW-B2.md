# HW-B2: Hailo bring-up

Status: PROPOSED. **Conditional.** Human bench work; can start now.

| Field | Value |
|---|---|
| Objective | The stock person-detection app runs on the real HAT, with versions recorded |
| Owner | B |
| Branch | Evidence through a human commit on `codex/HW-B2` |
| Depends on | Nothing |
| Budget | Until Sat 18:00. Pi OS and Hailo packages from their official sources |

## Scope
Follow [Raspberry Pi's AI guide](https://www.raspberrypi.com/documentation/computers/ai.html):

1. Check the OS is Trixie 64-bit. Update as documented.
2. **Find out which accelerator we have.** Use `hailo-all` for AI Kit or AI HAT+, `hailo-h10-all` for AI HAT+ 2. They cannot coexist.
3. Reboot, then run `hailortcli fw-control identify`, and `dmesg | grep -i hailo`.
4. Clone [hailo-apps](https://github.com/hailo-ai/hailo-apps) and run `./install.sh`. **Its docs say most recently tested with HailoRT 5.4.0.** Compare with the version `hailo-all` installed. If mismatched, try the matching tag and record it.
5. `hailo-detect --input rpi`. Find out how to run without a screen if needed.
6. Install `paho-mqtt` in that app's venv.

## Explicit exclusions
No custom models. No Dataflow Compiler. No ONNX on Hailo.

## Allowed edits
None in the repo apart from `docs/evidence/HW-B2/*`.

## Pre-approved commands and runs
The documented commands above, on the Pi, by B.

## Acceptance (evidence file `docs/evidence/HW-B2/report.md`)
Identify output (architecture, firmware). `hailortcli --version`. Package names and versions. hailo-apps commit or tag.
HEF used. Measured FPS. Pi temperature after 10 minutes. Camera model and its actual field of view.

## Risks
Version mismatch. Camera used by another process. Pi power supply too weak.

## Stop conditions
Not detecting people by Sat 23:00: tell D and use the plan's fallback.

## Handoff and close
Report to D. Tell the SW-07 builder of any API differences. Human commits evidence by explicit path.
