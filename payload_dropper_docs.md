# Payload Dropper (SITL / Gazebo) — Implementation & Reproduction Guide

Adds a droppable payload to an `x500`-style PX4 model in Gazebo, triggered by the
**real** `MAV_CMD_DO_GRIPPER` command chain (the same MAVLink command a real drone
uses). This document is a self-contained recipe for reproducing it on a fresh PX4
checkout.

**7 changes across 3 areas**, done in the order below.

---

## How it works

```
MAVLink MAV_CMD_DO_GRIPPER  ─▶  payload_deliverer module  ─▶  uORB "gripper" topic (GRAB/RELEASE)
   (or mission "release" item)      (already in PX4)                    │
                                                                        ▼
                                             [ shim in gz_bridge ] ─▶ gz topic ".../detach"
                                                                        │
                                                                        ▼
                                             gz DetachableJoint system releases the payload
                                                    (coke_can falls under gravity)
```

Everything left of the shim is stock PX4 flight code. The shim + `DetachableJoint`
are the **simulation-only** substitute for a physical release servo (see
[Sim vs. real hardware](#sim-vs-real-hardware) at the end).

---

## Prerequisites

- PX4 SITL builds with Gazebo (`make px4_sitl gz_x500` works).
- The `gz-sim-detachable-joint-system` plugin exists (ships with gz Harmonic / gz-sim8):

  ```bash
  ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/ | grep detachable
  ```

---

## 1. New Gazebo model — `Tools/simulation/gz` (git submodule)

> Note: `Tools/simulation/gz` is a **git submodule**. These two files live in the
> submodule, not the main PX4 repo — commit them there (or in your fork of it).

`Tools/simulation/gz/models/x500_mono_cam_down_payload/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>x500_mono_cam_down_payload</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Your Name</name><email>you@example.com</email></author>
  <description>X500 with downward mono camera and a droppable payload.</description>
</model>
```

`Tools/simulation/gz/models/x500_mono_cam_down_payload/model.sdf`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_mono_cam_down_payload'>
    <self_collide>false</self_collide>

    <include merge='true'><uri>x500</uri></include>
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>0 0 .10 0 1.5707 0</pose>
      <name>mono_cam</name>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
    </joint>

    <!-- Payload (coke_can is a stock PX4 gz model; its link is named "link") -->
    <include>
      <uri>model://coke_can</uri>
      <name>payload</name>
      <pose>0.1 0 -0.05 0 0 0</pose>
    </include>

    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>payload</child_model>
      <child_link>link</child_link>
      <!-- No <detach_topic> on purpose: the plugin defaults to
           /model/<runtime_model_name>/detachable_joint/detach, which carries the
           _N instance suffix the bridge shim also uses. Never hardcode it. -->
    </plugin>
  </model>
</sdf>
```

> ⚠️ **Biggest gotcha:** do NOT set `<detach_topic>` to a hardcoded model name. PX4
> spawns the model as `<name>_<instance>` (e.g. `..._0`), so a hardcoded topic
> without the suffix silently never matches — `Publish()` still succeeds, so it
> *looks* like it works. Omitting it lets the plugin derive the correct runtime name.
> No `server.config` change is needed — this is a model plugin, not a world system.

`coke_can` is a stock PX4 gz model already present in `Tools/simulation/gz/models/` —
no need to create it.

---

## 2. Airframe file

`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_mono_cam_down_payload`

```sh
#!/bin/sh
#
# @name Gazebo x500 mono cam down with payload
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mono_cam_down_payload}

. ${R}etc/init.d-posix/airframes/4001_gz_x500

# Gripper machinery: type 0 = Servo enables it (this is already the default).
# There is NO PD_GRIPPER_EN param — do not add one.
param set-default PD_GRIPPER_TYPE 0
```

> Pick the next free `40xx` number for your repo — `4022` may already be taken on a
> different PX4 version.

---

## 3. Register the airframe

In `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, add to the
`px4_add_romfs_files(...)` list:

```cmake
	4022_gz_x500_mono_cam_down_payload
```

---

## 4. Bridge shim — header

`src/modules/simulation/gz_bridge/GZBridge.hpp`

Add the include (near the other `uORB/topics/*`):

```cpp
#include <uORB/topics/gripper.h>
```

Add the members (in the `private:` section, by the other subscriptions):

```cpp
uORB::Subscription _gripper_sub{ORB_ID(gripper)};
gz::transport::Node::Publisher _payload_detach_pub;
bool _payload_detached{false};
```

---

## 5. Bridge shim — implementation

`src/modules/simulation/gz_bridge/GZBridge.cpp`

In `init()`, alongside the other `_node.Advertise(...)` calls:

```cpp
// Match the DetachableJoint default topic; _model_name carries the _N instance suffix.
std::string detach_topic = "/model/" + _model_name + "/detachable_joint/detach";
_payload_detach_pub = _node.Advertise<gz::msgs::Empty>(detach_topic);
```

In `Run()`, before the final `ScheduleDelayed(10_ms);`:

```cpp
gripper_s gripper;
if (_gripper_sub.update(&gripper)) {
	if (gripper.command == gripper_s::COMMAND_RELEASE && !_payload_detached) {
		gz::msgs::Empty msg;
		if (_payload_detach_pub.Publish(msg)) {
			_payload_detached = true;
			PX4_INFO("Payload released");
		}
	}
}
```

> - `gz::msgs::Empty` is already available via the `<gz/msgs.hh>` include at the top
>   of the header.
> - `_payload_detached` latches after the first release. `DetachableJoint` has no
>   `<attach_topic>` here, so the payload cannot re-attach anyway — **to drop again,
>   restart the sim.**
> - Mind the tabs — run `make check_format` before committing; PX4 CI enforces style.

---

## 6. Trigger script

`Tools/simulation/payload_release_pymavlink.py` — sends `MAV_CMD_DO_GRIPPER`
(param1 = gripper instance, param2 = `0` RELEASE / `1` GRAB) on UDP `14540+id`, and
waits through the `IN_PROGRESS` ack the `payload_deliverer` emits before the final
`ACCEPTED`.

```bash
pip install pymavlink
# take off first (other terminal / QGC), then:
python3 Tools/simulation/payload_release_pymavlink.py            # drone 0, instance 1, RELEASE
python3 Tools/simulation/payload_release_pymavlink.py --drone-id 2
python3 Tools/simulation/payload_release_pymavlink.py --grab     # send GRAB instead
```

Alternatively, add a "Release payload" mission item in QGroundControl — same command.

---

## 7. Build, run, verify

```bash
make px4_sitl gz_x500_mono_cam_down_payload
```

Before flying, confirm the two topics line up (this is what caught the original bug):

```bash
gz topic -l | grep detach
# → /model/x500_mono_cam_down_payload_0/detachable_joint/detach
```

That is exactly what the shim publishes to (`/model/` + `_model_name` +
`/detachable_joint/detach`). In the pxh shell, confirm the command path with
`listener gripper`. Take off, then run the trigger script — expect
`INFO [gz_bridge] Payload released` and the can drops.

---

## Troubleshooting quick-map

| Symptom | Cause | Fix |
|---|---|---|
| `Payload released` prints but nothing drops | detach topic name mismatch (hardcoded vs runtime `_0`) | omit `<detach_topic>`; build shim topic from `_model_name` |
| Payload falls at spawn | `<child_model>`/`<child_link>` name wrong → joint never created | match nested model name (`payload`) and its link (`link`) |
| ACK "not accepted" (`result=5`) | that's `IN_PROGRESS`, not failure | wait past it (script already does) |
| Gripper never acts | expected a `PD_GRIPPER_EN` param | it doesn't exist — use `PD_GRIPPER_TYPE 0` |
| CI format failure | astyle indentation | `make check_format` |
| Can't drop twice in one session | `_payload_detached` latch + no `<attach_topic>` | restart the sim |

---

## Sim vs. real hardware

Items **1, 4, 5** (the model + the `gz_bridge` shim + `DetachableJoint`) are
**simulation-only**. A real drone has no detach topic — a physical servo moves a
latch.

On real hardware you instead:

1. Keep `PD_GRIPPER_TYPE = 0` (servo).
2. In the QGC **Actuators** tab (or `PWM_AUX_FUNCn` / actuator params), assign a
   servo output (e.g. AUX1) to output function **Gripper (430)**.
3. Set that output's min/max/disarmed PWM so the endpoints match latched / released.
4. `FunctionGripper` drives the output to `+1` on GRAB and `-1` on RELEASE — the
   servo swings and the payload drops.

Items **2, 3, 6** (airframe params, airframe registration, MAVLink trigger) carry
over unchanged.

> Note: the `DetachableJoint` sim path bypasses the actuator/mixer output entirely
> (uORB → shim → gz), so it validates command/mission logic but **not** the servo
> output mapping. To exercise the exact hardware code path in sim, use the servo-latch
> variant instead: set `SIM_GZ_SV_FUNC1 = 430` and add a `servo_0` revolute joint with
> a `JointPositionController` plugin to the model (see the `standard_vtol` model for
> the pattern).
