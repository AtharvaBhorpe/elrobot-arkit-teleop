# Raspberry Pi 3 Wireless Onboard Controller — Design

**Date:** 2026-07-28  
**Status:** Proposed; approved design discussion, awaiting written-spec review

## Goal

Move the hardware-facing part of Elrobot onto a Raspberry Pi 3 Model B v1.2 so
the arm, bus-servo adapter, and two USB cameras can be operated over the existing
Wi-Fi router. Keep ARKit reception, IK, the browser cockpit, and LeRobot dataset
recording on the workstation.

ROS 2 remains the application API. `rmw_zenoh` replaces DDS for the distributed
ROS graph, allowing the existing ROS nodes and safety boundary to remain intact.

## Decisions

- Use Raspberry Pi OS Lite 64-bit based on Debian Trixie.
- Use the existing Raspberry Pi 3 Model B v1.2; do not use the ESP32.
- Connect the Waveshare Bus Servo Adapter (A) to a Pi USB port in USB mode.
- Connect both cameras by USB and request native MJPEG at 640x480, 15 FPS.
- Put the Zenoh router and the hardware driver on the Pi.
- Keep phone reception, IK, cockpit, camera decoding, and recording on the
  workstation.
- Use `rmw_zenoh_cpp` end-to-end rather than a DDS-to-Zenoh bridge or a custom
  native-Zenoh control gateway.
- Use Pixi on both machines, with separate workstation and onboard
  environments in one lock file.
- Connect both computers to the existing Wi-Fi router using explicit Zenoh
  endpoints.
- Do not add Tailscale in this phase.

Raspberry Pi OS is preferred over Ubuntu 26.04 because the Pi kernel, firmware,
V4L2 camera support, USB behavior, and boot integration remain host-OS
responsibilities even when application packages live in Pixi. Raspberry Pi OS
supports this Pi generation directly and leaves more of the Pi's 1 GB RAM for
the robot processes.

## Non-goals

- Rewriting ROS nodes to use the native Zenoh API.
- Moving ARKit reception, IK, the cockpit, or LeRobot encoding onto the Pi.
- Recording a dataset on the Pi.
- Streaming raw camera frames over Wi-Fi.
- Internet exposure, router port forwarding, or remote access outside the LAN.
- Changing calibration JSON, servo EEPROM, the kinematic URDF, joint limits,
  workspace bounds, velocity limits, or the manipulability threshold.
- Automatically starting the torque-enabled driver after a Pi reboot.

## System architecture

```text
 iPhone
 ARKit / UDP
     │
     ▼
 Workstation
 ┌─────────────────────────────────────────────────────────────────────┐
 │ arkit_receiver → IK ──────────────────────── /joint_command         │
 │ browser cockpit                         ◀─── /joint_states          │
 │ compressed-camera decoder              ◀─── camera JPEG topics     │
 │ LeRobot recorder ◀── local raw images + joint state/command         │
 │ rmw_zenoh client                                                    │
 └──────────────────────────┬──────────────────────────────────────────┘
                            │ Wi-Fi LAN, Zenoh TCP
                            ▼
 Raspberry Pi 3
 ┌─────────────────────────────────────────────────────────────────────┐
 │ rmw_zenohd router                                                   │
 │ elrobot_driver ── USB serial ── Waveshare adapter ── servo bus      │
 │ wrist camera publisher ── USB UVC                                  │
 │ external camera publisher ── USB UVC                               │
 └─────────────────────────────────────────────────────────────────────┘
```

The Pi is the Zenoh router because the physical control boundary lives there.
The local driver remains connected to its router even when the workstation or
Wi-Fi disappears. The workstation is an explicit router client; discovery does
not depend on multicast crossing the router.

There remains exactly one ROS graph and one robot. Topic names stay compatible
with the current application.

## Component placement

### Raspberry Pi

The Pi runs:

- `rmw_zenohd`
- one `elrobot_driver` process, the only owner of the servo serial device
- one wrist-camera process, the only owner of its V4L2 device
- one external-camera process, the only owner of its V4L2 device
- lightweight health reporting and systemd supervision

The Pi does not run Pinocchio IK, RViz, Foxglove, FastAPI, LeRobot datasets, or
video encoders.

### Workstation

The workstation runs:

- `arkit_receiver`
- Cartesian IK
- the web cockpit and collection manager
- one decoder for each remote compressed camera stream
- the existing LeRobot recorder and exporter
- optional visualization tools

The decoder republishes raw images under the existing topic names, so the
cockpit and recorder continue to consume normal `sensor_msgs/Image` messages.

## ROS and Zenoh transport

Both hosts set:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

The Pi starts `rmw_zenohd` before any robot ROS nodes. Its router listens on a
fixed LAN TCP endpoint. The workstation uses a checked-in client configuration
whose connect endpoint is supplied from deployment configuration, normally the
Pi's DHCP reservation or stable hostname. The endpoint is not hard-coded in
Python.

The LAN firewall permits the configured Zenoh TCP port only on the local
network. No Zenoh listener is exposed through the internet router.

Zenoh shared memory is disabled. It provides no cross-host benefit and consumes
unnecessary memory on the 1 GB Pi.

### Distributed topics

| Topic | Type | Direction | Rate | QoS |
|---|---|---|---:|---|
| `/joint_command` | `sensor_msgs/JointState` | workstation → Pi | up to 100 Hz | best effort, keep last 1 |
| `/joint_states` | `sensor_msgs/JointState` | Pi → workstation | driver rate | best effort, keep last 1 |
| `/wrist_cam/image/compressed` | `sensor_msgs/CompressedImage` | Pi → workstation | 15 FPS | best effort, keep last 1 |
| `/ext_cam/image/compressed` | `sensor_msgs/CompressedImage` | Pi → workstation | 15 FPS | best effort, keep last 1 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Pi → workstation | 1 Hz or on change | reliable, keep last 1 |

The following high-rate topics remain workstation-local:

- `/target_pose`
- `/wrist_cam/image`
- `/ext_cam/image`
- collection and replay internals

`/record/cmd` and `/record/status` remain reliable and local because both the
cockpit and recorder stay on the workstation.

Keep-last-one, best-effort command delivery is intentional: an old target is
never more useful than the newest target, and middleware must not build a
motion backlog during congestion.

## Command freshness and deadman behavior

`rmw_zenoh` does not currently provide all ROS deadline and lifespan QoS
features. Command freshness is therefore enforced inside the existing hardware
safety boundary.

Every `/joint_command` publisher must set `header.stamp`. The Pi driver:

1. rejects a command with a zero timestamp;
2. rejects a command older than the existing 200 ms deadman interval;
3. rejects a command timestamp more than 50 ms in the future;
4. does not refresh its local deadman timer for a rejected command; and
5. updates its monotonic arrival timer only after the command passes freshness
   and the existing safety checks.

The 50 ms future bound is an admission check for clock faults, not a mechanical
safety threshold. It is one quarter of the existing deadman window and must be
confirmed by the Wi-Fi soak test before real motion. It may only be widened from
measured clock-offset and jitter evidence.

Both hosts use network time synchronization. The onboard start procedure
refuses torque-enabled distributed operation unless measured workstation/Pi
clock offset is within 50 ms. A clock step or loss of synchronization causes
commands to be rejected and the existing deadman to freeze the arm.

The current 200 ms deadman, slew clamp, velocity clamp, workspace check,
manipulability check, joint limits, and grasp latch remain otherwise unchanged.
After a communication freeze, the next fresh, safe command resumes the current
behavior; this phase does not add a new arming state to the driver.

## Camera path

Each Pi camera process opens one stable V4L2 device and requests:

```text
pixel format: MJPEG
resolution:   640x480
frame rate:   15 FPS
```

The publisher reads the camera's encoded JPEG frame and places those bytes
directly into `sensor_msgs/CompressedImage` with `format="jpeg"`. It must not
decode and re-encode frames on the Pi. The message timestamp is the capture
timestamp.

On the workstation, the decoder:

- subscribes with best-effort, keep-last-one QoS;
- decodes only the newest JPEG;
- preserves the source header and capture timestamp; and
- publishes BGR8 `sensor_msgs/Image` on the existing raw topic.

This preserves the current cockpit and recorder interface while keeping raw
640x480 BGR traffic off Wi-Fi. If a subscriber or the network falls behind,
frames are dropped rather than queued.

Camera devices use `/dev/v4l/by-id` when the camera exposes a unique serial.
Otherwise deployment uses `/dev/v4l/by-path` or a physical-port udev alias.
`/dev/video0` and `/dev/video1` are not accepted as persistent configuration.

The two cameras should be attached through a powered USB hub, while the servo
adapter remains directly connected to the Pi. The hub solves camera power
budget risk; it does not create extra USB bandwidth. All three devices still
share the Pi's USB 2.0 host controller, so the soak test is a release gate.

### Recording integrity

The remote deployment profile records at 15 FPS. A managed recording may start
only after both camera streams and joint state are present and less than one
second old. During an episode, a camera stream older than one second is a
recording error:

- arm control continues;
- stale frames are not silently duplicated into the dataset;
- the cockpit reports which stream failed; and
- the incomplete episode is not committed as a valid episode.

A dropped individual JPEG is normal and does not stop recording. The recorder
continues sampling the newest pair at its configured rate while both feeds are
fresh.

## USB servo adapter

The Waveshare adapter is used in its USB-host configuration, with the adapter's
servo power wired exactly as required by the existing arm setup. The Pi USB
connection supplies communication, not servo-bus power.

Deployment uses a stable `/dev/serial/by-id` path. The process user belongs to
`dialout`; it does not run as root. There is only one driver process and it is
the only process allowed to open the adapter.

The driver retains the existing serial recovery behavior. If camera
renegotiation disrupts the shared USB controller, the driver freezes at the
last slew-limited mini-goal while recovering. Repeated serial recovery, USB
resets, or driver-loop overruns appear in diagnostics and fail the soak gate.

GPIO UART remains a fallback only if measurement shows that the Pi's shared USB
path cannot run the adapter and both cameras reliably. It is not part of the
first implementation.

## Pixi environments

The repository keeps one workspace and lock file with two target platforms:

- `linux-64`
- `linux-aarch64`

It defines two environments:

### `workstation`

Contains the current complete stack: ROS 2 Jazzy, `rmw_zenoh_cpp`, Pinocchio,
LeRobot dataset support, FastAPI, RViz/Foxglove and camera decoding tools.
Existing workstation tasks continue to run through this environment.

### `onboard`

Contains only what the Pi needs: Python, ROS 2 Jazzy base packages,
`rmw_zenoh_cpp`, `sensor_msgs`, `diagnostic_msgs`, the Feetech/LeRobot bus
dependency, and the V4L2 compressed-camera publisher dependencies. It excludes
the dataset, web, visualization, and IK stacks.

The lock is solved and committed from the workstation for both platforms. The
Pi installs the locked `onboard` environment; it does not independently
re-resolve dependency versions.

An onboard import gate must prove the driver, Feetech bus, ROS messages,
`rmw_zenoh_cpp`, and camera publisher imports on AArch64 before any hardware
service is started.

## Service supervision and boot behavior

Systemd owns the Pi processes:

- the Zenoh router is enabled at boot;
- the two camera services may start at boot;
- the torque-enabled robot driver is installed but not enabled at boot;
- an `elrobot-onboard.target` starts the hardware services after an operator
  completes the physical preflight and supports the arm;
- each hardware service has a distinct device and process;
- services restart on failure with rate limiting; and
- camera failure or restart never restarts the driver.

The driver service orders itself after the Zenoh router and time-sync checks.
It starts by reading present positions, setting goals to those positions, and
then enabling torque, matching the current no-jump startup contract.

Stopping the driver retains the current behavior: torque stays on holding. A Pi
power loss removes torque and the arm goes limp. Software cannot make that safe;
the physical setup must support the arm against an unexpected power loss.

Logs go to the system journal with bounded retention. Diagnostics report, at
minimum:

- Zenoh connection state;
- driver command age and frozen/running state;
- driver-loop overrun and serial-recovery counters;
- stable serial device identity;
- per-camera frame age, rate, dropped-frame count, and device identity;
- Pi memory use, temperature, and throttling state; and
- workstation/Pi clock offset admission result.

## Failure behavior

| Failure | Required behavior |
|---|---|
| Workstation exits | command silence; Pi driver freezes within the existing 200 ms deadman |
| Wi-Fi disconnects | same as workstation exit; no queued motion on reconnect |
| Zenoh router exits | driver receives no commands and freezes; systemd restarts router independently |
| Old command arrives after reconnect | timestamp rejection; it does not refresh the deadman |
| Pi clock is unsynchronized | driver service does not enter distributed torque-enabled operation |
| One or both cameras fail | motion is unaffected; new recording is refused and an active incomplete episode is not committed |
| Camera subscriber falls behind | old frames are dropped; control messages are not queued behind video |
| Servo USB transaction fails | existing recovery runs; driver retains its last safe mini-goal and reports the fault |
| Camera service fails | only that service restarts; driver remains running |
| Driver restarts | present position is read and adopted before torque is enabled |
| Pi loses power | servos become limp; physical support is required |

## Verification

All automated tests remain offline-safe. Any test that creates ROS nodes sets
`ROS_DOMAIN_ID=77`, even though production uses `rmw_zenoh`.

### Automated checks

- Existing `pixi run test` suite passes in the workstation environment.
- Driver safety tests cover fresh, stale, zero-stamped, and future-stamped
  commands, and prove rejected messages do not refresh the deadman.
- All command sources, including cockpit sliders and replay, produce stamped
  commands.
- Camera publisher tests wrap a known JPEG without pixel transformation.
- Workstation decoder tests preserve header timestamps and output the expected
  BGR image.
- Recorder tests reject missing or stale camera streams and do not commit an
  incomplete episode.
- QoS tests prove command and camera queues are best-effort keep-last-one.
- Environment tests solve and install both `linux-64` and `linux-aarch64`
  profiles from the committed lock.

### Hardware rollout gates

1. Install Raspberry Pi OS Lite 64-bit, configure the existing Wi-Fi router,
   SSH, stable hostname/address, and time synchronization.
2. Install the locked onboard Pixi environment and pass its import gate.
3. With servo torque disabled and the arm supported, identify stable paths for
   the bus adapter and both cameras.
4. Verify both cameras deliver native 640x480 MJPEG at 15 FPS without the
   driver running.
5. Run the driver with cameras stopped, at minimum configured velocity, and
   verify no-jump startup and joint-state feedback.
6. With the arm supported and stationary, start and stop each camera while the
   driver holds position. Any unexplained serial error or USB reset fails the
   gate.
7. Run all three USB devices plus Zenoh for 30 minutes. Record camera rates and
   drops, serial errors, command age, memory, CPU temperature, throttling, Wi-Fi
   throughput, and clock offset.
8. Send a synthetic command stream, disconnect Wi-Fi, and prove the driver
   freezes within 200 ms without replaying queued commands after reconnection.
9. Repeat the disconnect test on the real arm at minimum velocity with physical
   support and an operator at the power cutoff.
10. Record a short two-camera episode on the workstation and verify frame
    timestamps, frame counts, joint alignment, dataset reload, and replay.
11. Increase to normal operating velocity only after every preceding gate
    passes.

The release gate requires zero unexplained serial failures, USB resets,
driver-loop deadline misses, Pi throttling events, or stale-command
acceptances during the 30-minute run. Camera frame drops are acceptable only
when they do not create a one-second stale interval or invalidate the recorded
episode.

## Fallback order

If the hardware gates fail, change one variable at a time and repeat the soak:

1. reduce both cameras to 10 FPS;
2. confirm the powered hub and Pi power supply under load;
3. move the servo adapter from USB to the Pi GPIO UART while retaining the same
   driver and safety boundary;
4. replace the Pi 3 with a newer single-board computer if CPU, RAM, Wi-Fi, or
   USB measurements still miss the gates.

Do not compensate for missed deadlines by widening the command deadman or
weakening driver safety thresholds.

## Deferred work

- Tailscale or another remote-access overlay.
- Authentication and authorization beyond the trusted LAN.
- A Pi-hosted access point.
- Adaptive camera bitrate or resolution.
- Hardware power-loss braking or an uninterruptible power supply.
- A native-Zenoh protocol or non-ROS embedded controller.

## References

- [Raspberry Pi OS documentation](https://www.raspberrypi.com/documentation/computers/os.html)
- [Raspberry Pi hardware documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- [Pixi multi-platform workspaces](https://pixi.sh/latest/workspace/multi_platform/)
- [RoboStack ROS 2 Jazzy](https://robostack.github.io/jazzy.html)
- [`rmw_zenoh` for ROS 2](https://github.com/ros2/rmw_zenoh)
- [Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))
