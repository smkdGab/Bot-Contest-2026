# Hardware setup and safety

Run from `BOTticelli/` with the virtual environment active.
Connect your computer to the K10's Wi-Fi and use the IP shown on its display.

```bash
python -m botticelli hardware --host YOUR_BOT_IP shell
python -m botticelli hardware --dry-run servo positional angle S0 45
```

See [CLI.md](CLI.md) for commands. In the shell, use `help servo positional angle` or
`servo positional angle --help`; `exit` closes the session.
Positional commands default to type 270 and home 0. Use `servo continuous` for
continuous servos on S0..S5, and `dc-motor` for DC motors on M1..M4.

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
An unconfirmed step names the operation, channel, action code and reply error.
An earlier stop error is cleared only after a later stop for that channel is acknowledged.

**Network stopping is not a physical guarantee.** Motion leases attempt stop/home
commands when control stalls, but SIGKILL, computer power loss or lost Wi-Fi can
prevent cleanup. Treat heartbeat timeout and `STOP_ALL_MOTORS` conservatively as
DC-only stops; neither releases master ownership. ACKs are not physical feedback.

## References

See the contest's [bot capabilities](../ai-capabilities.md),
[sensor prerequisites](../ai-sensors.md) and [protocol](../binary-protocol.md).
Sound playback pauses several sensors and shock polling; do not rely on shock
protection while audio is active.
