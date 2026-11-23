# Repository Map

Bobik is spread over several independent git repositories, checked out as siblings in `~/projects/bobik/`. **`bobik_robot` is the overarching repository**: Claude is always started here, and the siblings are made visible through `additionalDirectories` in [`.claude/settings.json`](../.claude/settings.json). The repositories stay separate (no monorepo, no submodules). Interfaces between them are documented here instead.

```
~/projects/bobik/
├── bobik_robot/     ← main repo: ROS 2 package, URDF, docs, plan   (git@github.com:slesinger/bobik_robot)
├── bobik_arduino/   ← Arduino Mega firmware                         (slesinger/bobik_arduino)
├── bobik_driver/    ← Jetson TK1 hardware driver                    (slesinger/bobik_driver)
└── bobik_web/       ← phone web UI                                  (slesinger/bobik_web)
```

`HOPEJr` also sits in that directory. It is a third-party arm design and is **not part of Bobik**. Ignore it.

## Repositories

| Repo | Language / toolchain | Runs on | Owns |
|---|---|---|---|
| `bobik_robot` | Python, ROS 2 (ament), URDF | Workstation `ha.doma` | Robot model (`urdf/`, `meshes/`), launch files, Nav2/SLAM params, voice skills (`bobik_robot/skills`), `state_publisher`, all design docs (`docs/`, `docs-ai/`) |
| `bobik_arduino` | C++, PlatformIO (atmelavr) | Arduino Mega 2560 | 20 Hz frame loop, caster control, IR-tick odometry input, IMU read, **the serial protocol definition** (`lib/protocol/protocol_types.h`), pin map (`lib/robot/robot_config.h`) |
| `bobik_driver` | C++14, CMake, ZeroMQ, boost::asio | Jetson TK1 (L4T 21.3, upstart service) | UART ⇄ Arduino (`/dev/ttyTHS1`, 115200 in code), XV-11 lidar reader, Kinect sender, ZMQ link to workstation, Jetson installation notes in its README |
| `bobik_bridge` | C++, ROS 2 Foxy (colcon), ZeroMQ | Workstation `ha.doma` | ZMQ ⇄ ROS translation: subscribes `/cmd_vel`; publishes `/odom`, `/joint_states`, `/scan`, `/imu`, `driver/raw/caster`. Computes odometry (`src/bobik_bridge.cpp`). ZMQ endpoints are hard-coded: radio → `udp://192.168.1.21:7654`, dish bound on `udp://*:7655` |
| `bobik_interfaces` | ROS 2 IDL (ament_cmake, rosidl) | Workstation (build dependency of `bobik_robot`; used by `bobik_web` via rosbridge type names) | Custom `.msg`/`.srv`/`.action` types. Currently only copies of std_msgs-style types plus the ROS tutorial `AddTwoInts` and `Fibonacci` |
| `bobik_web` | React 17, roslib, npm | Served from workstation, used on a phone (A52s) | Dashboard UI: control, map, camera, chat, STT, action buttons. HTTPS certs in `cert/` |

## Interfaces between repositories

```
bobik_web ──wss / rosbridge──► ROS 2 (bobik_robot + Nav2 + bridge) ──ZeroMQ UDP──► bobik_driver ──UART──► bobik_arduino
```

| Link | Mechanism | Defined in | Notes |
|---|---|---|---|
| Arduino ⇄ Jetson | UART frames with CRC | `bobik_arduino/lib/protocol/protocol_types.h` | **Single source of truth.** `bobik_driver/src/protocol_types.h` is a symlink to it, using an absolute path `/home/honza/projects/bobik/bobik_arduino/...`, so the two repos must stay at these paths. Byte-level description: [software-legacy.md](software-legacy.md) §2. The README in `bobik_arduino` describes an older frame layout (`$AA` start byte), so trust the header and the legacy doc. |
| Jetson ⇄ workstation | ZeroMQ RADIO/DISH (draft API) over UDP | `bobik_driver/src/bobik_driver.cpp`, `bobik_bridge/src/bobik_bridge.cpp` | Hard-coded IPs and ports (see [software-legacy.md](software-legacy.md) §1). The workstation end is `bobik_bridge`. |
| Workstation ⇄ phone | rosbridge websocket (TLS) | `bobik_interfaces` (types), `bobik_web/src/*.js`, `bobik_robot/launch/rosbridge_websocket_launch.xml` | The web UI calls ROS services `/human_query`, `/bobik_say`, `/bobik_poweroff`, `/motor_relay`. Services are implemented in `bobik_robot/bobik_robot/bobik_robot.py` (stubs that only speak). Certificate paths are hard-coded in `bobik_web/README.md`. |

## Gaps in the picture

- **`bobik_interfaces` is out of sync with its users.** `bobik_robot.py` imports `bobik_interfaces.srv.HumanQuery` and `bobik_web/src/App.js` calls `bobik_interfaces/srv/HumanQuery`, but `srv/` contains only `AddTwoInts.srv` and no `HumanQuery.srv` exists in the checkout. The repo has a single commit ("first commit"), so the definition was either never committed or lives elsewhere. As it stands, `bobik_robot` would not build or import against this checkout.

- The plan ([plan.md](plan.md), Phase 2) replaces the driver stack. When that happens, update this map, especially the interface table.

## Working across repositories

- Start Claude in `bobik_robot`. The three siblings are then readable and editable in the same session.
- Each repo is committed separately. A change that crosses a boundary (for example a protocol change) needs one commit per repo, and the commit messages should name the matching change in the other repo.
- Protocol changes start in `bobik_arduino/lib/protocol/protocol_types.h`, then `bobik_driver`, then the bridge.
- Documentation lives only in `bobik_robot/docs-ai/` and `bobik_robot/docs/`. The sibling READMEs hold build and installation notes only.
