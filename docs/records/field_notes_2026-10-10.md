# Field Notes — 2026-10-10 (FMTC on-site hardware day)

Notes from the first real-hardware validation day. Kept here so anyone on the
team (or a fresh dev machine) can get this context without re-discovering it
the hard way.

## TL;DR

- **First real driving test (forward/backward) worked.** Arduino Mega + motors
  driven via WSL2 + `usbipd`, capped with `serial_sender`'s `th_scale` param
  for safety.
- **First real camera-based lane detection worked** — but only on a
  **native Ubuntu** machine, not in any VM/WSL2.
- **Camera video over WSL2 or VirtualBox is a dead end on laptop hardware.**
  Don't spend more time trying to make `usb_cam` work inside a VM — see
  verdict below.
- Not yet validated: steering (`angular.z`) with real Arduino, `sign_detect`
  on real footage, lidar-dependent nodes (`obs_evade`/`gap_follow`/`wall_follow`)
  on real `/scan`, full `s1_stack.launch.py` integrated run.

## Camera streaming verdict: needs native Linux

Spent most of the day trying to get 2× Logitech C920 streaming through
**WSL2 (`usbipd attach --wsl`)** and separately through **VirtualBox USB
passthrough**, on the same Windows laptop. Lidar (RPLiDAR A2M8) and Arduino
(low-bandwidth serial) eventually worked in both after enough retries, but
camera video never worked reliably in either one:

- `usb_cam_node_exe` either crashed (`terminate called after throwing an
  instance of 'char*'`) or ran with ~0 throughput (`Failed to send AVPacket
  to decode`, `Select timeout, exiting...`).
- Tried: different resolutions, different QoS (reliable vs. best-effort),
  ruling out USB hub power starvation (bought the correct 5V/3A hub adapter,
  confirmed via a portable power station — didn't help).

Two independent virtualization approaches failing identically, on exactly the
one thing both virtualize the same way (isochronous USB video), and nothing
else, is strong evidence this is a structural limitation of virtualizing UVC
webcam video on this laptop — not a config bug worth chasing further.

**Confirmed fix**: same `usb_cam` → `lane_trace` → `decision_auto` chain ran
immediately on a teammate's native (dual-boot) Ubuntu 22.04 machine, real
image data in, real steering output out.

**Takeaway: any node that touches a real camera needs to run on native
Linux.** Lidar/serial-only nodes are fine under WSL2 if a VM is ever needed
again for bench testing.

## VirtualBox gotchas (if you ever go back to it)

- USB 2.0/3.0 passthrough needs the Extension Pack installed separately
  (`VBoxManage extpack install`, admin, needs `--accept-license=<hash>`).
- Windows' own class driver grabs the device (COM port driver for Arduino,
  camera driver for webcams) before VirtualBox can capture it — run
  `Disable-PnpDevice` on that specific Windows driver instance first. The
  device's VBox UUID usually changes on every disable/replug cycle, so
  re-run `VBoxManage list usbhost` right before attaching.
- Attach frequently fails with `is busy with a previous request` for no
  clear reason. Fix order that worked empirically: retry same UUID a few
  times → unplug/replug the device → unplug/replug the whole hub →
  `VBoxManage controlvm <vm> reset` → full `poweroff` + `startvm`.
- **Never restart the `VBoxSVC` Windows service while a VM is running** —
  it force-powers-off the VM immediately.
- Don't script keyboard input into the guest (`keyboardputstring`/
  `keyboardputfile`) — there's no way to pause for a `sudo` password prompt,
  and a stray keystroke can land on the wrong window if focus shifts. Set up
  Guest Additions + bidirectional clipboard instead and type/paste normally.

## Debugging trap: a terminal that "looks alive" isn't evidence

`usb_cam`, `lane_trace`, and `decision_auto` all died silently more than once
today (crash or just exit) while their terminal windows still showed old
scrollback that looked like live output (e.g. repeated `STALE INPUT -
STOPPING` lines from a process that had already exited). The only reliable
check is live state, not remembered terminal text:

```bash
ros2 node list                                  # is it actually registered right now
ros2 topic info <topic> --verbose                # real current pub/sub counts
```

## `usb_cam` namespace gotcha

`usb_cam_node_exe` publishes to plain `/image_raw` unless you remap its
namespace:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/cam_lane
```

`lane_trace`/`sign_detect` subscribe to the **namespaced** topic
(`/cam_lane/image_raw` or `/cam_front/image_raw`). Forgetting the remap means
`usb_cam` runs with zero errors and `lane_trace` silently never receives
anything — no crash, no warning, just permanent silence. Easy to misdiagnose
as a deeper vision bug. Always check with `ros2 topic info <topic>
--verbose` (publisher/subscriber counts) before concluding a vision node
"isn't detecting anything."

## Safe first real-driving test

`serial_sender` has a `th_scale` param that globally caps drive speed without
touching code — used for today's first real forward/backward test:

```bash
ros2 run s1_stack serial_sender --ros-args -p th_scale:=200   # default is 1000
```

## Fresh machine setup gotchas

Don't assume a teammate's "already configured" machine actually is. Today's
native Ubuntu machine was missing, despite being assumed ready:

- ROS2 apt repo not configured at all
  (`/etc/apt/sources.list.d/ros2.list` missing).
- `ros-humble-usb-cam`, `cv_bridge`, `joy` not installed.
- `pip`/`pyserial` not installed.
- No WiFi hardware detected by NetworkManager — worked around with USB-C
  tethering.

When relaying setup commands to someone typing by hand with no working
clipboard, send **one short command at a time** — multi-line blocks and
one-liners with `$(...)`/pipes/quotes reliably get mistyped.

## Still not validated (next steps)

- Steering (`angular.z`) with the real Arduino — only forward/backward
  tested so far.
- `sign_detect` against real camera footage.
- `obs_evade`/`gap_follow`/`wall_follow` against real `/scan` data.
- Full `s1_stack.launch.py` integrated run (today's tests were deliberately
  split across machines/pieces, not combined yet).
