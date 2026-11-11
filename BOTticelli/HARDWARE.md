# Hardware setup and safety

Run from `BOTticelli/` with the virtual environment active.
Connect your computer to the K10's Wi-Fi and use the IP shown on its display.

```bash
python -m botticelli hardware --host YOUR_BOT_IP shell
python -m botticelli hardware --dry-run servo angle S0 45 --type 180 --home 90
```

See [CLI.md](CLI.md) for commands. In the shell, use `help servo angle` or
`servo angle --help`; `exit` closes the session.

## Before motion

Follow the [wiring](../ai-wiring.md) and [mandatory checklist](../ai-mandatory-checklist.md).
Confirm servo types, safe endpoints, home positions and speed limits on an
unloaded mechanism. Provide a physical means to cut actuator power.
For real driving, set these in [config/bot.yaml](config/bot.yaml) and enable
its confirmation fields only after calibration. Example values are not calibration.

## Session behavior

Every live session registers once and privately prompts for the TFT token,
including reads and sound. Never store the token in commands, config or logs.
Help, capabilities and dry-run are offline.

Cleanup sends speed zero individually to configured continuous servos, then
best-effort `STOP_ALL_MOTORS`, homes explicitly configured positional servos,
disables streams and unregisters. Servo reads do not infer a type or home.
Ctrl+C, SIGTERM and exceptions follow this path; cleanup failures are reported.

**Network stopping is not a physical guarantee.** Motion leases attempt stop/home
commands when control stalls, but SIGKILL, computer power loss or lost Wi-Fi can
prevent cleanup. Treat heartbeat timeout and `STOP_ALL_MOTORS` conservatively as
DC-only stops; neither releases master ownership. ACKs are not physical feedback.

## References

See the contest's [bot capabilities](../ai-capabilities.md),
[sensor prerequisites](../ai-sensors.md) and [protocol](../binary-protocol.md).
Sound playback pauses several sensors and shock polling; do not rely on shock
protection while audio is active.
