# Phase 0 Log (bring-up and baseline)

Chronological record of what was done and measured during M0. Facts about the hardware belong in [hardware/](hardware/); this file is the history. Plan: [plan.md](plan.md#phase-0--bring-up-and-baseline-no-purchases).

## 2026-11-23

**Battery**
- Pack voltage 13.6 V (about 3.4 V per cell) after 2–3 years of storage. Looks like new, no swelling. See [hardware/electrical.md](hardware/electrical.md#battery-state-2026-11-23).
- First charge through the robot's own charge path (adapter → manual switch → Arduino relay → battery), supervised by hand. After about 30 min: 15.8 V unloaded. Real capacity and BMS behaviour at full are still to be determined.

**Jetson first boot**
- Powered from the 230 V adapter, WiFi only, at 192.168.1.21. SSH with key works as user `ubuntu` (host `tegra-ubuntu`).
- System: Ubuntu 14.04.6, kernel 3.10.40-grinch-21.3.4, L4T R21.3, 1.9 GB RAM (350 MB used), eMMC 14.7 GB (root 14 GB, 39 % used), temperatures 25–29 °C. `bobik_driver` starts on boot (upstart job `bobik`).

**Backup of the eMMC**
- Full image of `/dev/mmcblk0`, taken over WiFi with `dd | gzip -1`, with `bobik_driver` stopped during the backup and started again afterwards.
- File: `~/backups/bobik/tegra-emmc-2026-11-23.img.gz` on the workstation (3.4 GB compressed, 15,758,000,128 bytes uncompressed, `gzip -t` OK). SHA-256 in `tegra-emmc-2026-11-23.img.gz.sha256`: `3dedb668dc528535130c4ce60d90c1961f11268939b5a8f9ac29aaf0098b3369`.
- Taken from the running system, so the root file system may be slightly inconsistent. Restore with `zcat … | sudo dd of=/dev/mmcblk0` from a recovery boot. Not yet tested.

**Findings on the compute platform (2026-11-23)**
- Jetson TK1 right now: **no CUDA toolkit and no OpenCV installed**. `libfreenect` is built (`~/libfreenect/build/bin`: freenect-camtest, freenect-regtest, freenect-glview, fakenect-record, …). USB: a QinHeng HL-340 USB-serial adapter (XV-11 lidar); the Kinect is not connected yet. `nproc` shows 1 (the TK1 keeps cores off when idle), CPU at 1.8 GHz.
- Workstation `ha`: ASUS MINIPC PN50, AMD Ryzen 5 4500U (6 cores, Zen 2, 15 W class), 30 GB RAM, Radeon Vega (Renoir) iGPU, no NVIDIA GPU, Realtek gigabit Ethernet, no WiFi card listed by `lspci`. 78 % / and 89 % /home full. Not an option for the robot itself (it is the workstation), but the PN50 class is a candidate type for an onboard computer: x86, runs ROS 2 and libfreenect, no CUDA, needs 19 V (battery is 13–16.8 V, so a boost converter).
- Jetson Orin Nano Super dev kit: NVIDIA list price went $249 → $399 in July 2026 (DRAM costs), backorder. Honza sees $560 at retail. That is above the whole 10,000 CZK budget. Sources: cnx-software 2026-07-22, notebookcheck.
- Decision tendency (Honza): stay on the old L4T system for now, connect Kinect (or any camera) as soon as possible and explore on-board feature extraction. Idea: run visual orientation (keypoints) on the robot and send only detected features to navigation, to cut WiFi traffic. Keep the CUDA option open: CUDA is not installed, so save the CUDA 6.5 and OpenCV4Tegra packages for L4T R21 before the archive disappears.

**Decisions and preferences (Honza, 2026-11-23)**
- No powered USB hub. Earlier, connecting the Kinect straight to the Jetson's USB port filled the USB bus and caused transfer errors. To be investigated when the Kinect is connected (controller, isochronous bandwidth, depth-only or lower RGB, other port).
- Security is not a concern (robot only on the local network).
- Time limit for fighting old tools is counted in problems, not days: no pushing through at any cost.
- A different workstation would be bought separately, so the PN50 stays as the workstation.
- Wanted: CUDA to take load off the CPU (feature extraction). Possible later replacement of the Arduino Mega by something faster but still real-time (Teensy 4.1 class, or the Circle bare-metal library on a Raspberry Pi).
- Price check 2026-11: NVIDIA raised Jetson prices in July 2026 (Orin Nano Super kit $249 → $399, Jetson Nano module $99 → $199 and the 4 GB Nano kit is being discontinued). Raspberry Pi 5 prices went up three times since December 2025, Orange Pi 5 (RK3588S) is about $200–270 with volatile prices. Teensy 4.1 is about $32–36.

**Kinect v1 first test (2026-11-23, old L4T system, `freenect-camtest`, 40 s)**
- Detected: Xbox NUI Camera (045e:02ae, 480M), Audio (02ad, 480M), Motor (02b0, full speed). The Kinect hangs on the Jetson's full-size USB-A port, which is behind an NEC 3-port high-speed hub on controller `tegra-ehci` (bus 2). **All three USB controllers in this kernel are `tegra-ehci` (USB 2.0, 480 Mbit/s); there is no xHCI/USB 3.** The blue port is therefore USB 2 here. The XV-11 USB-serial (CH340) is on bus 3, a separate controller.
- Result at 640×480, both streams: **video 1180 frames and depth 1176 frames in 40 s (about 29.5 fps each, timestamps 30 fps)**. Packet loss is low: depth 9 lost packets in 875 frames (0.01 per frame). Bursts of lost packets or "Invalid magic" occurred at stream start, once about 29 s in, and at shutdown. No sustained failure.
- Load: `freenect-camtest` (receive and unpack only, no processing) takes about 55–70 % of one core; total CPU about 60 % user + 19 % system while `bobik_driver` was also running. The TK1 keeps cores offline when idle (`nproc` = 1, `/proc/cpuinfo` lists 4). Little headroom for per-frame processing on one core, so core hotplug/governor has to be checked.
- The kernel module `gspca_kinect` is loaded and binds the camera interface (`/dev/video0`). libfreenect detached it by itself and worked.

**Kinect modes and CPU settings (2026-11-23)**
- libfreenect (v1 driver) offers video only at 640×480 and 1280×1024; **there is no 320×240 mode**, and depth is 640×480 only. Lower resolutions have to be made by downscaling in software. YUV/RGB-converted video modes at 640×480 run at 15 fps in the mode table; Bayer/IR modes run at 30 fps.
- CPU: cpufreq governor `interactive` (available: interactive, conservative, ondemand, powersave, userspace, performance), max 2.32 GHz. cpuquiet governor `runnable` keeps cores 1–3 **offline** when idle (cpu0 online, cpu1–3 offline at the time of the check). Candidate runtime change for tests: governor `performance` and cpuquiet off. Reverts at reboot.

**Packages for the CUDA door (2026-11-23)**
- Both packages are still downloadable from NVIDIA (`developer.download.nvidia.com`, page "Linux For Tegra R21.3" lists them with md5): CUDA 6.5 Toolkit for L4T R21.3 (`cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb`, 566 MB, md5 7652f8afcd01e5c82dc5c202a902469a) and OpenCV4Tegra 2.4.10.1 for L4T 21.x (`libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb`, 20.6 MB, md5 830eb30bbf6d5dbaa43358f14fc6139b).
- Saved on the workstation in `~/backups/bobik/packages/` with checksums. Known problem from the NVIDIA forum: apt may reject the local CUDA repo because of a weak SHA1 signature ("Broken CUDA repo for L4T 21.3").

**CUDA 6.5 and OpenCV4Tegra installed on the Jetson (2026-11-23)**
- What was changed on the Jetson: installed `cuda-repo-l4t-r21.3-6-5-prod` and `libopencv4tegra-repo` (NVIDIA local apt repos, their GPG keys added with `apt-key`), then `cuda-toolkit-6-5` (nvcc 6.5.35, samples, docs), `libopencv4tegra` and `libopencv4tegra-dev` 2.4.10.1 (apt also pulled in fakeroot and a few dev libs). User `ubuntu` was added to group `video` (needed for /dev/nvhost-*; without it `deviceQuery` returns error 38 "no CUDA-capable device"). Package files are in `~/packages`, a built `deviceQuery` in `~/cudasamples`. Disk: 7.2 GB used, 5.8 GB free.
- No SHA1 problem with the CUDA repo. The `apt-get update` errors about the deadsnakes PPA (403) are old and unrelated.
- **`deviceQuery` passes: GK20A, compute capability 3.2, 192 CUDA cores, 852 MHz, 1889 MB shared memory, CUDA 6.5.** `nvcc` needs `-ccbin g++-4.8` because the default `gcc` is the self-built gcc 9.4; `nvcc` is not in PATH yet (`/usr/local/cuda-6.5/bin`).
- Runtime CPU changes (lost at reboot): governor `performance`, cpuquiet disabled, cores 1–3 online, all four at 2.32 GHz. Not made persistent yet.

**Kinect test 2 with all cores at 2.32 GHz (`freenect-camtest`)**
- About 29.6 fps for both video and depth (889 video, 890 depth frames in about 30 s), **0 lost packets**.
- `freenect-camtest` now takes only **about 20 % of one core**, and total CPU is about 5 %. The 55–70 % of the first test was caused by the `interactive` governor and the idle cores, not by the Kinect. `bobik_driver` idles at about 1 %.

## Leap Motion vs Kinect (2026-11-23, answer from knowledge, not measured)

- Honza has an old Leap Motion Controller (v1) that gets warm when on. Question: could it replace the Kinect as a 3D sensor?
- **No, not as a Kinect replacement.** It has two IR cameras (stereo, baseline about 40 mm) and three IR LEDs, but it is built for hand tracking at roughly 2.5-60 cm (up to about 80 cm), with a wide ~150 deg field of view. Stereo depth from such a short baseline is useless beyond about 1 m. Kinect v1 gives a real structured-light depth map at 0.5-4.5 m (640x480, 30 fps), which is what navigation and obstacle detection need.
- The heat is normal (LEDs plus the onboard processor); it is not a sign of a better sensor.
- The software side is worse too: the official SDK gives hand/finger tracking and raw IR images, not a depth map or point cloud. Linux support of the SDK is limited (estimate, not verified) and would be hard on the ARM Jetson TK1.
- Possible niche use: close-range hand gestures at the robot (mounted on the head or body, facing the person). This is an optional idea, not part of the plan.
- Better depth upgrade candidates, if the Kinect v1 is ever replaced: Intel RealSense D4xx or an OAK-D (both USB 3; the TK1 has only USB 2, see the USB note above).

**Kinect recording and feature detection benchmark (2026-11-23)**
- Recording `table-static-1`: Kinect lying on the table, static scene, 10 s with `fakenect-record` into `/dev/shm` (RAM disk). 289 RGB frames (PPM, 640×480×3, 921,615 B each), 286 depth frames (PGM, 16-bit, 614,417 B), audio chunks, `INDEX.txt` and `device.json` (calibration). About **46 MB/s** of raw data (1.5 MB per frame pair at 30 fps), more than the USB rate I first estimated. Saved on the workstation as `~/backups/bobik/kinect/table-static-1.tar.gz` (210 MB, `gzip -t` OK). The copy in `/dev/shm` on the Jetson disappears at reboot.
- Benchmark `scripts/jetson/feat_bench.cpp` (OpenCV4Tegra 2.4.10.1, `cv::FAST`, `cv::ORB` 500 features, `cv::gpu::FAST_GPU`, `cv::gpu::ORB_GPU`), 200 frames from that recording, performance mode, single thread, GPU numbers include upload and download. Build: `g++-4.8 -O2 feat_bench.cpp -o feat_bench $(pkg-config --cflags --libs opencv) -L/usr/local/cuda-6.5/lib`, run with `LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib`.

| Test | Mean per frame | Rate | Keypoints |
|---|---|---|---|
| FAST, CPU, 640×480 | 0.98 ms | ~1020 fps | 364 |
| ORB 500, CPU, 640×480 | 27.2 ms | 36.7 fps | 500 |
| ORB 500, CPU, 320×240 | 9.1 ms | 110 fps | 410 |
| FAST, GPU, 640×480 | 2.4 ms | 417 fps | 364 |
| ORB 500, GPU, 640×480 | 55.2 ms | 18.1 fps | 500 |

- **The GPU was slower than one CPU core for FAST and ORB.** Caveats: the GPU clock was not locked (it may scale down), the CUDA 6.5 / OpenCV 2.4 `gpu` ORB is old, transfers are included (the TK1 has unified memory and zero-copy could be used), and only one static scene was tested. The test is therefore not final for the CUDA question. Heavier jobs (descriptor matching, optical flow, dense processing) are not measured yet.
- **Correction (same day):** during that first run the GPU was at its **minimum clock of 72 MHz** (`/sys/kernel/debug/clock/gbus/rate`), so the GPU numbers above are not valid. See the second benchmark below.
- `scripts/jetson/perf-mode.sh on|off` switches CPU performance mode and now also locks the GPU clock at 852 MHz (runtime only; `override.gbus` in debugfs).

**Benchmark 2: CPU time and concurrency, GPU clock locked at 852 MHz (2026-11-23)**
`scripts/jetson/feat_bench2.cpp`, 60 frames of `table-static-1`. "CPU" = CPU time of the process while the call runs. GPU numbers include upload and download. The CUDA wait mode (default or blocking sync) made no difference.

| Operation | CPU only: wall / CPU | With GPU: wall / CPU | CPU time saved |
|---|---|---|---|
| ORB 500 (640×480) | 26.2 / 26.2 ms | 40.6 / 33.3 ms | none (worse) |
| FAST | 1.0 / 1.0 ms | 1.6 / 1.0 ms | none |
| Match 500×500 Hamming | 4.5 / 4.5 ms | 2.0 / 0.36 ms | **92 %**, 2.3× faster |
| LK optical flow, 400 points | 10.0 / 10.0 ms | 7.5 / 3.3 ms | **67 %**, 1.3× faster |

Concurrency (10 s each, 4 cores): one CPU ORB thread 38.6 fps; GPU ORB thread alone 25.2 fps using 81 % of a core; CPU thread and GPU thread together 37.4 + 23.5 fps (**CPU and GPU do run in parallel**, the CPU thread loses only 3 %); two CPU threads 37.9 fps each.
- OpenCV 2.4 `ORB_GPU` still burns about 80 % of a core, so it does not free the CPU. Descriptor matching and sparse optical flow do offload most of their CPU time.
- Example at 30 fps with 500 features (ORB + match + LK): CPU only about 40.5 ms per frame (about 1.2 cores); ORB on CPU with match and LK on GPU about 29.6 ms (about 0.9 core). A saving of about a quarter to a third of a core. The CPU-only variant already fits in the four cores. Matching cost grows with the square of the feature count, so the GPU gains more with more features.

## Leap Motion as end-effector camera and gestures (2026-11-23, answer from knowledge, not measured)

- Honza: the human-robot communication still has to be solved; finger gestures should not be ruled out, but a simpler solution may come first. When the arm is finished, a camera on the end-effector will be needed. Is Leap suitable?
- **Range fits, everything else is weak.** Its working range (about 2.5-60 cm) is the range a gripper camera cares about, and a 40 mm stereo baseline gives usable depth at roughly 10-40 cm. Against it: IR-only grayscale (no colour, so no object recognition by colour or VLM), strong pulsed IR LEDs that can saturate on shiny close objects, a heavily distorted wide-angle image that needs its own calibration, a vendor service that is hard to run on the ARM TK1, and raw images are not a depth map (stereo would have to be computed by us). Size is about 80x30x11 mm, so it is not tiny on a small gripper.
- **Verdict:** not recommended as the end-effector camera. It would only make sense as an experiment, because Honza already owns it and it costs nothing.
- Better candidates for a wrist camera (not verified, prices not checked): Intel RealSense D405 (made for 7-50 cm, RGB + depth, USB 3), a small OAK-D Lite / OAK-D SR, or a plain small USB RGB camera (endoscope or ArduCam style, USB 2 friendly) plus a ToF distance sensor such as VL53L5CX. The plain RGB camera is the cheapest and works with the TK1's USB 2.
- Simpler communication ideas to compare with gestures (not decided): phone/web UI (already planned), voice commands (planned), a few coarse gestures from the Kinect body skeleton or the RGB image (raised hand, pointing), ArUco/QR cards shown to the camera, physical buttons on the robot. Leap finger gestures stay as an optional later experiment.

## Power consumption check (2026-11-23)

- Question: can the Jetson report its own power draw? Checked read-only over SSH (run in the tmux session `bobik`).
- **No onboard power sensor.** The `ina3221x` driver is registered in the L4T kernel, but no device is bound to it. I2C address 4-0040 on bus 4 is the AS3722 PMIC, not an INA3221. `hwmon` only has thermal sensors (MEM-therm, PLL-therm, Tboard_tegra, Tdiode_tegra).
- `~/tegrastats` works but on TK1 it only shows RAM, CPU load, VDE and EDP limit, with no power figures.
- So the current draw has to be measured outside the Jetson: a USB/DC inline power meter on the 230 V adapter output (or a smart plug), or a shunt/INA219 on the battery line. Until then only estimates are possible. Not decided.

## Next

- Test Kinect (libfreenect, already on the Jetson) on the old system.
- Flash `bobik_arduino` and run the old stack once.
- Baseline per subsystem, then the square test.
