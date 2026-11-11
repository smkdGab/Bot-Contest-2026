# Python primitives

The CLI and real driver use the same standard-library `Robot` API.
Configure only the connected servo channels and use calibrated limits.

```python
from botticelli.primitives import Robot, private_token, termination_cleanup

with termination_cleanup():
    with Robot("YOUR_BOT_IP", private_token()) as bot:
        bot.configure_servo(0, 180, home=90, minimum=30, maximum=120)
        bot.servo_angle(0, 45)
        acceleration = bot.read_imu()   # accel_mg: [x, y, z]
```

The context registers once and cleans up on exit. See [hardware safety](HARDWARE.md).
For continuous servos, use `configure_servo(5, 'continuous')` and
`servo_speed(5, 20)`; cleanup sends speed zero.

| Device | Methods |
|---|---|
| Servos via DFRobot DFR1216 (S0–S5) | `configure_servo`, `servo_angle`, `servo_increment`, `servo_speed`, `servo_read` |
| Motors via DFRobot DFR1216 (M1–M4) | `motor_speed`, `motor_read`, `stop` |
| LiDAR — DFRobot SEN0682 64x8 DTOF | `read_lidar`, `read_intensity`, `reset_lidar` |
| IMU — DFRobot SEN0409 LIS2DW12 accelerometer | `read_imu`, `imu_config`, `configure_imu`, `reset_imu`, `imu_stream_state`; impacts: `read_bump`, `bump_config`, `configure_bump` |
| Compass — DFRobot SEN0619 BMM350 magnetometer | `read_compass` |
| Vision — DFRobot SEN0305 HuskyLens K210 | `read_camera`, `camera_algorithm`, `set_camera_algorithm`, `camera_controls`, `camera_illumination`, `camera_stream_state` |
| LEDs — K10 NeoPixels / DFR1216 WS2812 | `led_set/off/read`, `board_led_set/off/read` (each is a separate method) |
| Display — UniHiker K10 TFT | `screen('app-info')`, `screen('next')`, `screen('previous')` |
| Sound — UniHiker K10 speaker | `sound_list/status/upload/delete/play/stop/volume` (separate methods) |
| Connection — UniHiker K10; battery via DFR1216 | `ping`, `name`, `battery` |

Motion calls accept `ttl` (default 0.5 s, maximum 60 s); renew by issuing fresh
motion calls. Expiry faults the session and attempts known stop/home commands.
`stop()` stops propulsion; positional homing happens at session exit.
There is no automatic reconnect or replay of expired motion.

Use `stream('imu', hz=10)`, `latest('imu')` and
`stream('imu', enabled=False, hz=10)` for telemetry. Names: `lidar`,
`intensity`, `imu`, `bump`, `compass`, `camera`. These are output keys: IMU and
bump share one accelerometer and stream; distance and intensity share one LiDAR.
The CLI groups these under their devices; Python method and stream names stay unchanged.
`latest` returns `None` after `max_age` (default 0.5 s).
Fresh streamed reads accept `timeout=2`; absent bump events return `None`,
while other sensor timeouts fault the session. Cleanup errors are recorded in
`cleanup_errors` and raise on otherwise successful context exit.

For signatures, use `help(Robot.servo_angle)` or inspect
[primitives.py](botticelli/primitives.py). Bot features are described in the
[contest capability reference](../ai-capabilities.md).
