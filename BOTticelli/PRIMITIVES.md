# Python primitives

The CLI and real driver use the same standard-library `Robot` API.
Configure only the connected servo channels and use calibrated limits.

```python
from botticelli.primitives import Robot
from botticelli.safety import private_token, termination_cleanup

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
| Servos | `configure_servo`, `servo_angle`, `servo_increment`, `servo_speed`, `servo_read` |
| Motors (1–4) | `motor_speed`, `motor_read`, `stop` |
| LiDAR | `read_lidar`, `read_intensity`, `reset_lidar` |
| IMU | `read_imu`, `imu_config`, `configure_imu`, `reset_imu`, `imu_stream_state` |
| Bump | `read_bump`, `bump_config`, `configure_bump` |
| Compass | `read_compass` |
| HuskyLens | `read_camera`, `camera_algorithm`, `set_camera_algorithm`, `camera_controls`, `camera_illumination`, `camera_stream_state` |
| LEDs | `led_set/off/read`, `board_led_set/off/read` (each is a separate method) |
| Display | `screen('app-info')`, `screen('next')`, `screen('previous')` |
| Sound | `sound_list/status/upload/delete/play/stop/volume` (separate methods) |
| Connection | `ping`, `name`, `battery` |

Motion calls accept `ttl` (default 0.5 s, maximum 60 s); renew by issuing fresh
motion calls. Expiry faults the session and attempts known stop/home commands.
`stop()` stops propulsion; positional homing happens at session exit.
There is no automatic reconnect or replay of expired motion.

Use `stream('imu', hz=10)`, `latest('imu')` and
`stream('imu', enabled=False, hz=10)` for telemetry. Names: `lidar`,
`intensity`, `imu`, `bump`, `compass`, `camera`. IMU and bump share a stream.
`latest` returns `None` after `max_age` (default 0.5 s).
Fresh streamed reads accept `timeout=2`; absent bump events return `None`,
while other sensor timeouts fault the session. Cleanup errors are recorded in
`cleanup_errors` and raise on otherwise successful context exit.

For signatures, use `help(Robot.servo_angle)` or inspect
[primitives.py](botticelli/primitives.py). Bot features are described in the
[contest capability reference](../ai-capabilities.md).
