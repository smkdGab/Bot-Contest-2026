
# CAUTION

**This repo has been generated is  90% AI generated.**
**If you face issues with code, contact the contest organizer**


---







# K10 Bot Contest: Participant Documentation

Welcome to the aMaker bot contest documentation. This repository contains the
participant guides, firmware protocol references, AI coding instructions, and
Python client examples for the UniHiker K10 bots supplied at the event.
It is the documentation workspace, not the firmware source tree.

Browse the [participant website](https://CSE-Amadeus-Amaker-Club.github.io/Bot-Contest-2026/)
or follow the Markdown links below directly on GitHub. The website starts at
[index.html](index.html).

## Participants: Start Here

1. Read the [contest guide](bot-contest-guide.md) for supplied parts, build
   constraints, arena format, and hardware safety. Confirm timing, scoring, and
   event-specific procedures with the organizers.
2. Follow the [wiring reference](ai-wiring.md) before powering the bot.
3. Use the [board UI guide](board-ui.md) to find the bot's network address,
   master token, sensor screens, and logs.
4. Read the [quickstart](quickstart.md) and
   [mandatory client checklist](ai-mandatory-checklist.md) before motion control.
5. Choose a client below, or build your own using the
   [UDP client guide](ai-udp-client.md).

Keep the bot's master token out of chat history, source code, logs, and committed
configuration. Read it from the screen and supply it at runtime.

## Find the Right Reference

| Need | Reference |
|---|---|
| Contest rules, assembly, and safety | [Participant contest guide](bot-contest-guide.md) |
| First connection and control session | [Quickstart](quickstart.md) |
| Wiring and power connections | [Wiring](ai-wiring.md) |
| Screen navigation and diagnostics | [Board UI](board-ui.md) |
| Required session lifecycle and cleanup | [Mandatory checklist](ai-mandatory-checklist.md) |
| Supported hardware features and limits | [Capabilities](ai-capabilities.md) |
| Command groups and access requirements | [Services](ai-services.md) |
| Command bytes, payloads, and replies | [Binary protocol](binary-protocol.md) |
| UDP and HTTP transport behavior | [Communication](communication.md) |
| Runnable controller patterns | [UDP client guide](ai-udp-client.md) |
| Sensor prerequisites and decoding context | [Sensors](ai-sensors.md) |
| Sound conversion, storage, and playback | [Sound tutorial](ai-udp-client.md#sound-convert-store-and-play) |

## Have a loot at demo Python Client

| Client | Use it for |
|---|---|
| [SensorsScreen](python_example/sensorscreen/README.md) | View HuskyLens, LiDAR, IMU, and geomagnetic data on a computer; access its documented servo, gamepad, and sound controls. |
| [BOTticelli](BOTticelli/README.md) | Drive a front-steered car with one gamepad path, using a bounded IR-SIM practice arena or the real K10 backend after calibration. |

Each client README owns its installation, configuration, launch, and offline test
instructions. Choose the client that matches your actual hardware; do not assume
the example's channel mapping matches your bot. BOTticelli's practice simulation
is a configurable approximation, not a hardware validation. Test new controller
code with mocks or loopback before connecting to hardware.

## AI Assistants: Read Before Coding

Start with the [repository instructions](AGENTS.md) and the
[AI agent guide](ai-agent-readme.md). Use the
[controller prompt template](ai-controller-prompt.md) to specify the task,
language/runtime, hardware channels and servo types, and network configuration.
[Participant agent instructions](participant-AGENTS.md) provide reusable guidance
for a participant's own controller project.

Before implementing a controller, read the mandatory checklist, binary protocol,
communication guide, UDP client examples, and the relevant wiring and sensor
pages linked above. When editing a Python client, also follow the scoped
instructions under [python_example](python_example/AGENTS.md) and, for
SensorsScreen, its [client instructions](python_example/sensorscreen/AGENTS.md).

- Active handlers in the sibling firmware repository's `include/services` and
  `src/services` are the protocol source of truth. Report documentation conflicts;
  without verifiable source or a documented payload, ask instead of guessing.
- Do not invent commands, endpoints, authentication headers, or capabilities.
  Client examples are secondary references, not protocol authority.
- Register before master-protected commands and keep an independent heartbeat
  sender running every 25-30 ms. Heartbeat timeout and `STOP_ALL_MOTORS` stop DC
  motors only; neither releases master ownership nor guarantees physical safety.
- On normal exit, exceptions, and interruption, send speed zero individually to
  every continuous servo configured by the client, then best-effort
  `STOP_ALL_MOTORS` before unregistering. Never replay stale motion after a timeout.
- Validate replies and use one receive dispatcher per socket, as specified in the
  checklist. Do not send live motion commands without explicit authorization.

Sound-only clients need no registration. Sound playback pauses shock detection
and several sensor/UI activities; do not rely on shock protection while audio is
active. Follow the sound tutorial for its complete limitations.

## Repository and Documentation Maintenance

The root Markdown files hold the guide content. Matching HTML pages provide the
website navigation and render that Markdown using [marked.min.js](marked.min.js).
Styles and images live alongside the guides; Python applications and their tests
live under `python_example/` and `BOTticelli/`.

Keep protocol tables in the binary protocol reference, runnable examples in the
client guide, and contest policy in the participant contest guide. Link to those
owners instead of maintaining duplicate instructions.

[sync-docs.sh](sync-docs.sh) compares selected mirrored help pages and assets with
the sibling firmware checkout. Run `bash sync-docs.sh --check` from this repository
root to inspect drift without writing. Review differences before using its
`--apply` mode. The participant contest guide and Python clients are maintained
here, not mirrored from firmware. Publishing changes is a separate, reviewed Git
operation; synchronization does not automatically commit or push.
