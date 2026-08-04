#!/usr/bin/env python3
"""
payload_impact_scorer.py - score a dropped payload against the M1 Abrams targets in
a PX4 SITL world by listening to the payload's contact sensor.

Piece 2 of the drop-accuracy pipeline: subscribe to the contact stream, filter by
collision name, debounce to one event per drop, print HIT/MISS + miss-distance, and
append one JSON record per drop for offline CEP analysis.
Piece 3 (--explode): on a HIT, set off an explosion burst and then a lingering fire
at the impact point.

Measurement philosophy - record truth, derive metrics offline:
  The scorer does NOT decide what "miss distance" means, because that depends on a
  reference it cannot know. Distance to the *aimpoint* is CEP; distance to the
  *nearest* target flatters the system (in military_recon_mini, aiming at abrams_0
  and landing on abrams_2 scores ~0 m); distance to the target that was *struck* is
  circular. So each record carries the true impact point and the true pose of EVERY
  target in the world, and the analysis picks the reference. Miss distance is
  recorded on hits too - a hit on the far edge of a hull is still a ~3 m error, and
  truncating those at the tank silhouette biases CEP low.

  The aimpoint is deliberately not sourced from the tracker at runtime: the tracker
  is the system under test, and a biased tracker grading its own drop reports itself
  as accurate. For a tracker-in-the-loop run, leave --aim-model unset and join the
  companion's aimpoint log to these records by timestamp offline. --aim-model /
  --aimpoint exist for ballistics-only runs with no tracker in the loop.

Normally you do NOT run this by hand: px4-rc.gzsim starts it in the background for
any `*payload*` airframe (see Tools/simulation/payload_dropper_docs.md). It is
launched right after the model is spawned, so it waits for the contact topic to
appear (--wait) instead of giving up, and it exits when the sim goes away
(--exit-with-sim) so it does not outlive the world it was scoring.

VFX approach - auto-emit on spawn, NOT topic triggering:
  The particle-emitter command topic (/model/.../emitter/cmd) is unreliable across
  gz-sim versions (works on 8.11, silently no-ops on 8.14). But a particle emitter
  with <emitting>true</emitting> fires the instant the model exists. So instead of
  spawning an armed emitter and triggering it, we SPAWN emitters that are already
  emitting: a one-shot explosion (emitting=true + a short <duration>) that bursts on
  creation, then the continuous `fire` model. No cmd topic is ever published.

Requirements: gz-transport13 + gz-msgs10 python bindings (apt: python3-gz-transport13).
The PROTOCOL_BUFFERS env var below is mandatory on this box: the apt gz-msgs10
protobuf stubs predate the pip protobuf 6.x, so the C++ impl rejects them.

The spawned effects need model://fire (and model://explosion's texture) resolvable by
the running server, i.e. gazebo_models on the sim's GZ_SIM_RESOURCE_PATH.

Usage:
    python3 payload_impact_scorer.py                       # score only
    python3 payload_impact_scorer.py --topic /world/.../contact
    python3 payload_impact_scorer.py --explode             # + explosion & fire on HIT
    python3 payload_impact_scorer.py --aim-model abrams_1  # ballistics-only run
    python3 payload_impact_scorer.py --world tank --model x500_payload_0 \
        --wait 60 --exit-with-sim --explode                # how px4-rc.gzsim calls it
"""
import os
# MUST be set before any gz.msgs (protobuf) import on this system.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import argparse
import json
import math
import re
import signal
import subprocess
import sys
import threading
import time
import warnings

warnings.filterwarnings("ignore")

from gz.transport13 import Node
from gz.msgs10 import contacts_pb2, pose_v_pb2, empty_pb2

# ---------------------------------------------------------------------------
# Configuration - match the scoped names seen in `gz topic -e` on the contact topic.
# ---------------------------------------------------------------------------
DEFAULT_WORLD = "tank"
PAYLOAD_TAG = "payload"          # substring identifying the payload side of a contact
TARGETS = {                      # scoped-name suffix -> label
    "hull_collision":   "hull",
    "turret_collision": "turret",
}
GROUND_TAG = "ground_plane"
# Model names counted as targets, matched with re.search so it covers both naming
# styles in the tree: tank.sdf includes the model under its own name "m1-abrams",
# while military_recon_mini.sdf renames three instances to abrams_0/1/2. The old
# exact-name compare against "m1-abrams" silently matched NOTHING in the multi-target
# world, so every drop there logged "tank pos unknown".
TARGET_PATTERN = r"abrams"

RECORD_SCHEMA = "payload_impact/1"

# --- geodetic origin ---
# WGS84, for converting the world's ENU metres to/from the lat/lon a companion
# computer logs. The Cartesian impact/target positions stay AUTHORITATIVE: they are
# exact truth, and CEP is a distance in metres, so nothing needs a projection to be
# computed. The origin is recorded so the *estimate* (GPS) can be brought into the
# truth frame - never the other way round, which would bake projection error into the
# reference. Derived lat/lon is emitted purely as a join convenience.
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

REARM_IDLE_SEC = 3.0             # re-arm if contacts go quiet this long (backup re-arm)

# --- sim liveness (--exit-with-sim) ---
# Poll the topic list rather than watching message flow: a GUI-paused sim stops
# publishing /clock and pose/info but keeps the topics advertised, and pausing must
# not look like a dead sim. This is the backstop for a sim that dies without
# signalling us (e.g. `pkill gz sim`); Ctrl-C is handled by install_signal_handlers().
SIM_POLL_SEC = 3.0
SIM_MISSES_TO_EXIT = 2

# --- logging ---
# The scorer normally runs inside the sim's console, so match PX4's format:
# "<LEVEL> [<module>] <message>", two spaces after INFO/WARN. flush because the
# console output is a pipe (tee) when auto-started.
TAG = "payload_scorer"


def info(msg):
    print(f"INFO  [{TAG}] {msg}", flush=True)


def warn(msg):
    print(f"WARN  [{TAG}] {msg}", flush=True)


def err(msg):
    print(f"ERROR [{TAG}] {msg}", flush=True)


def install_signal_handlers():
    """Die with the terminal that started the sim.

    px4-rc.gzsim starts us as a background job of a non-interactive shell, and POSIX
    has `sh` set SIGINT/SIGQUIT to SIG_IGN for async jobs - a disposition that
    survives exec. Without this, Ctrl-C in the sim's terminal stops PX4 and gz but
    NOT us, and we keep writing to a console whose prompt has already come back until
    the --exit-with-sim poll notices. Restoring the default disposition (plus SIGHUP /
    SIGTERM) makes the scorer go down with everything else.

    Exit is silent on purpose: the shell prompt is already back by then, so a farewell
    line would just land on top of it."""
    def _quit(_signum, _frame):
        os._exit(0)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _quit)
        except (ValueError, OSError):
            pass  # not on the main thread / unsupported - the poll backstop still applies


# --- explosion/fire VFX (spawned already emitting; no trigger topic needed) ---
FIRE_MODEL = "model://fire"      # continuous flame left burning at the impact point
FIRE_FUSE_SEC = 0.4              # delay after the boom before the fire lights, s
# Albedo sprite for the inline explosion; model://explosion or model://fire both work.
EXPLOSION_SPRITE = "model://explosion/materials/textures/puff.png"


def explosion_sdf(name):
    """A one-shot explosion: <emitting>true</emitting> so it bursts the instant it's
    spawned, and a short <duration> so it stops after the burst (particles then fade
    over <lifetime>). Single-quoted XML attributes so the whole string embeds cleanly
    in the create service's protobuf-text `sdf: "..."` field without escaping."""
    return (
        "<sdf version='1.9'>"
        f"<model name='{name}'><static>true</static><link name='link'>"
        "<particle_emitter name='emitter' type='point'>"
        "<pose>0 0 0 0 -1.57 0</pose>"
        "<emitting>true</emitting><duration>0.25</duration>"
        "<size>1 1 1</size><particle_size>10 10 10</particle_size>"
        "<lifetime>1.5</lifetime><min_velocity>5.0</min_velocity>"
        "<max_velocity>14.0</max_velocity><scale_rate>3.0</scale_rate><rate>500</rate>"
        "<color_start>1.0 0.5 0.1 1</color_start><color_end>0.1 0.1 0.1 1</color_end>"
        "<material><diffuse>1.0 0.6 0.2 1</diffuse><pbr><metal>"
        f"<albedo_map>{EXPLOSION_SPRITE}</albedo_map>"
        "</metal></pbr></material>"
        "</particle_emitter></link></model></sdf>"
    )


class Scorer:
    def __init__(self, world, drone_model, explode=False, target_pattern=TARGET_PATTERN,
                 record_path=None, run_id=None, aim_model=None, aim_xy=None,
                 origin=None):
        self.world = world
        self.origin = origin
        self.drone_model = drone_model
        # Contacts whose *other* side matches the carrier are self-contacts (payload
        # releasing from / brushing the drone) and are skipped without disarming, so
        # the first real impact against the tank or ground is what gets scored.
        self.ignore_tags = [drone_model] if drone_model else []
        self.explode = explode
        # Start DISARMED until the detach subscription is confirmed (see
        # set_detach_mode). The payload hangs below base_link and rests on the ground
        # plane while the drone is parked, so an initially-armed scorer books that
        # contact as a drop ~2.8 s into every run - a phantom "miss" tens of metres
        # out that would poison a CEP sample with one outlier per sim start.
        self.armed = False
        self.require_detach = True
        self.last_contact_t = 0.0
        self.target_re = re.compile(target_pattern)
        self.targets = {}            # model name -> live pose, from pose/info
        self.hit_count = 0
        self.drop_index = 0
        self.record_path = record_path
        self.run_id = run_id
        self.aim_model = aim_model
        self.aim_xy = aim_xy

    # --- helpers ----------------------------------------------------------
    @staticmethod
    def _other(contact):
        """Return the scoped name of the non-payload side of a contact."""
        a, b = contact.collision1.name, contact.collision2.name
        return b if PAYLOAD_TAG in a else a

    def _is_self(self, other):
        """True if the other side is the drone itself (release/brush contact)."""
        return any(tag in other for tag in self.ignore_tags)

    @staticmethod
    def _centroid(contact):
        """Average the (possibly many) contact points into one world-frame location."""
        pts = contact.position
        n = len(pts)
        return (sum(p.x for p in pts) / n,
                sum(p.y for p in pts) / n,
                sum(p.z for p in pts) / n)

    @staticmethod
    def _peak_force_z(contact):
        """Largest |normal force| across the contact points (impact strength proxy)."""
        return max((abs(w.body_1_wrench.force.z) for w in contact.wrench), default=0.0)

    @staticmethod
    def _model_of(scoped):
        """Owning model of a scoped collision name:
        "abrams_1::body::hull_collision" -> "abrams_1". This is what makes a hit
        attributable in a multi-target world - the collision *suffix* is identical
        across every tank instance, so it alone cannot say which one was struck."""
        return scoped.split("::", 1)[0]

    @staticmethod
    def _stamp(msg):
        """Sim time of a message, or None. Sim time (not wall time) is the join key
        against the ulog and the companion's tracking log."""
        try:
            return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        except AttributeError:
            return None

    @staticmethod
    def _heading_xy(q):
        """The model's local +X axis projected into the world XY plane, normalised.

        Deliberately not a ZYX yaw: tank.sdf includes m1-abrams with a 90-degree roll
        (`<pose>8 8 -0.2 1.57 0 0</pose>`) to stand the mesh up, so its extracted yaw
        is 0 no matter which way the hull actually points. The first column of the
        rotation matrix is the local +X axis in world coordinates, which tracks the
        hull's forward direction whatever the mesh convention. None if that axis is
        vertical, leaving nothing to project."""
        w, x, y, z = q
        fx = 1.0 - 2.0 * (y * y + z * z)
        fy = 2.0 * (x * y + w * z)
        n = math.hypot(fx, fy)
        return (fx / n, fy / n) if n > 1e-6 else None

    def classify(self, other):
        for suffix, label in TARGETS.items():
            if other.endswith(suffix):
                return "HIT", label
        if GROUND_TAG in other:
            return "MISS", "ground"
        return "OTHER", other

    # --- callbacks --------------------------------------------------------
    def on_pose(self, msg: pose_v_pb2.Pose_V):
        """Keep a live pose for EVERY model matching the target pattern.

        pose/info also carries links, scoped as "<model>::<link>"; only unscoped names
        are top-level models. Storing all of them rather than one is what makes a
        multi-target world measurable - which target was struck and which was aimed at
        are different questions, and both need every target's truth pose at impact.
        Keeping it live (rather than sampled once) is also what will make a moving
        target work: the pose snapshotted into the record is the one at impact."""
        for p in msg.pose:
            if "::" in p.name or not self.target_re.search(p.name):
                continue
            q = p.orientation
            self.targets[p.name] = {"x": p.position.x, "y": p.position.y,
                                    "z": p.position.z, "q": (q.w, q.x, q.y, q.z)}

    def set_detach_mode(self, detach_ok):
        """Pick the arming policy from whether the detach signal is actually available.

        Strict (detach_ok): score exactly one impact per commanded release. This is
        the mode that keeps pre-release ground contact out of the record.
        Fallback (no detach topic, or the subscription failed): arm immediately and
        lean on the idle re-arm, which is the pre-existing behaviour - records may
        then include contacts that were never drops, so it warns."""
        self.require_detach = bool(detach_ok)
        self.armed = not self.require_detach
        if self.require_detach:
            info("waiting for the detach command before scoring "
                 "(pre-release ground contact is not a drop)")
        else:
            warn("no detach signal: scoring any payload contact, including the "
                 "payload resting on the ground before release - records from this "
                 "run may contain phantom drops")

    def on_detach(self, _msg: empty_pb2.Empty):
        # A new drop was commanded -> arm for the next impact.
        self.armed = True
        info("armed: detach commanded, waiting for impact")

    def on_contact(self, msg: contacts_pb2.Contacts):
        self.last_contact_t = time.monotonic()
        if not self.armed:
            return

        # Find the first contact against something that ISN'T the drone. Contacts
        # with the drone (payload releasing/brushing on the way out) are skipped
        # WITHOUT disarming, so the real tank/ground impact is what gets scored.
        c = None
        for cand in msg.contact:
            if not self._is_self(self._other(cand)):
                c = cand
                break
        if c is None:
            return  # only self-contacts this message - stay armed

        other = self._other(c)
        kind, label = self.classify(other)
        x, y, z = self._centroid(c)
        force = self._peak_force_z(c)

        self.armed = False  # debounce: one event per drop until re-armed
        self.drop_index += 1

        rec = self._record(kind, label, other, x, y, z, force, self._stamp(msg))
        self._write_record(rec)

        near = rec["nearest"]
        nstr = (f"{near['horiz']:.2f} m from {near['model']}" if near
                else "no target pose yet")
        if not rec["targets"]:
            # No truth pose means no miss vector, so this record cannot contribute to
            # CEP. Say so at the time rather than letting the analysis discover a
            # record it has to drop.
            warn(f"no target matched /{self.target_re.pattern}/ in {self.world} - "
                 "this record carries no miss vector and is unusable for CEP")

        if kind == "HIT":
            self.hit_count += 1
            info(f"HIT {label} on {self._model_of(other)} "
                 f"at ({x:+.2f}, {y:+.2f}, {z:+.2f}), peak Fz {force:.1f} N, {nstr}")
            if rec.get("misassigned"):
                warn(f"struck {self._model_of(other)} but aimpoint was {self.aim_model}")
            if self.explode:
                # Run off-thread: the effect sleeps for the fuse and must not
                # block the transport callback.
                threading.Thread(target=self.detonate_and_ignite,
                                 args=(x, y, z), daemon=True).start()
        elif kind == "MISS":
            info(f"MISS ground at ({x:+.2f}, {y:+.2f}), {nstr}")
        else:
            info(f"contact with {other} at ({x:+.2f}, {y:+.2f}, {z:+.2f})")

        # Where it landed in WGS84, so the drop can be eyeballed against a companion's
        # GPS log or dropped into a map without converting by hand. Derived from the
        # world origin - the metres on the line above stay the authoritative
        # measurement, and CEP is computed from those, never from these degrees.
        imp = rec["impact"]
        if "lat" in imp:
            alt = (self.origin.get("elev") or 0.0) + z
            info(f"impact GPS {imp['lat']:.7f}, {imp['lon']:.7f} ({alt:.2f} m AMSL)")

    # --- scoring / actions ------------------------------------------------
    def _with_geo(self, d):
        """Add derived lat/lon to a dict that already carries world x/y.

        Convenience only - the Cartesian fields stay authoritative. CEP is computed in
        metres from x/y and never needs these; they exist so a record can be lined up
        against a companion log that only speaks lat/lon."""
        if self.origin:
            lat, lon = enu_to_geodetic(d["x"], d["y"], self.origin)
            d["lat"], d["lon"] = lat, lon
        return d

    def _target_rows(self, ix, iy):
        """Truth pose and miss vector for every target, sorted by name for stable
        diffs. The vector matters as much as the scalar: a constant offset (bias) and
        a spread (variance) are different bugs, and downrange/crossrange separates
        them further - release timing and ballistics show up downrange, tracking and
        yaw error show up crossrange."""
        rows = []
        for name, t in sorted(self.targets.items()):
            dx, dy = ix - t["x"], iy - t["y"]
            row = self._with_geo({"model": name, "x": t["x"], "y": t["y"],
                                  "z": t["z"], "q": list(t["q"]), "dx": dx, "dy": dy,
                                  "horiz": math.hypot(dx, dy)})
            h = self._heading_xy(t["q"])
            if h:
                row["downrange"] = dx * h[0] + dy * h[1]
                row["crossrange"] = -dx * h[1] + dy * h[0]
            rows.append(row)
        return rows

    def _aimpoint(self):
        """Where the drop was *supposed* to go, if this run declared it up front.

        None for tracker-in-the-loop runs by design - see the module docstring. The
        record still carries the impact and every target pose, so the aimpoint can be
        joined in offline from the companion's log without re-flying anything."""
        if self.aim_xy:
            return {"x": self.aim_xy[0], "y": self.aim_xy[1], "source": "cli"}
        if self.aim_model:
            t = self.targets.get(self.aim_model)
            if t:
                return {"x": t["x"], "y": t["y"], "model": self.aim_model,
                        "source": "model"}
            warn(f"aim model '{self.aim_model}' has no pose yet, aimpoint omitted")
        return None

    def _record(self, kind, label, other, x, y, z, force, sim_time):
        outcome = {"HIT": "hit", "MISS": "miss"}.get(kind, "other")
        struck = self._model_of(other) if kind == "HIT" else None
        rows = self._target_rows(x, y)
        nearest = min(rows, key=lambda r: r["horiz"], default=None)
        aim = self._aimpoint()

        rec = {
            "schema": RECORD_SCHEMA,
            "run_id": self.run_id,
            "drop_index": self.drop_index,
            "sim_time": sim_time,
            "wall_time": time.time(),
            "world": self.world,
            "carrier": self.drone_model,
            # Cartesian is authoritative; lat/lon is derived convenience so these
            # records join to a companion's GPS log without it knowing the transform.
            "origin": self.origin,
            "impact": self._with_geo({"x": x, "y": y, "z": z}),
            "peak_force_z_n": force,
            "outcome": outcome,
            "struck": ({"model": struck, "part": label, "collision": other}
                       if struck else None),
            "targets": rows,
            "nearest": ({"model": nearest["model"], "horiz": nearest["horiz"]}
                        if nearest else None),
            "aimpoint": aim,
        }
        if aim:
            rec["aim_error"] = {"dx": x - aim["x"], "dy": y - aim["y"],
                                "horiz": math.hypot(x - aim["x"], y - aim["y"])}
            # Right-on-target-but-wrong-target is a target-selection failure, not a
            # dispersion failure. Folding it into CEP as a small miss would hide it,
            # so it is flagged and reported as its own rate.
            if aim.get("model") and struck:
                rec["misassigned"] = struck != aim["model"]
        return rec

    def _write_record(self, rec):
        """Append-only JSONL so runs concatenate: one drop needs one sim restart (the
        gz_bridge detach latch), so a CEP sample of N is always N processes."""
        if not self.record_path:
            return
        try:
            with open(self.record_path, "a") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError as e:
            warn(f"could not append to {self.record_path}: {e}")

    def _create(self, req, what):
        """Call the world's factory service to spawn something. The CLI echoes the
        Boolean reply ("data: true") on stdout - capture it instead of letting it
        land in the sim console, and report failures PX4-style."""
        res = subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/create",
             "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", req],
            capture_output=True, text=True, check=False)
        if "true" in res.stdout:
            return True
        detail = (res.stderr or res.stdout).strip().replace("\n", " ") or "no reply"
        warn(f"failed to spawn {what}: {detail}")
        return False

    def detonate_and_ignite(self, x, y, z):
        """Piece 3: at the hit point, spawn a one-shot explosion (bursts on creation)
        and then, a beat later, a continuous fire. Both spawn ALREADY emitting, so no
        particle_emitter command topic is used - works the same on gz-sim 8.11 & 8.14.
        NOTE: model://fire (and the explosion sprite) must be resolvable by the running
        server, i.e. gazebo_models on the sim's GZ_SIM_RESOURCE_PATH."""
        boom = f"boom_{self.hit_count}"
        fire = f"fire_{self.hit_count}"
        z = max(z, 0.0)

        # 1) one-shot explosion, inline SDF, emitting=true -> bursts immediately.
        sdf = explosion_sdf(boom)
        if self._create(f'sdf: "{sdf}", name: "{boom}", '
                        f'pose: {{position: {{x: {x} y: {y} z: {z}}}}}', boom):
            info(f"detonated {boom}")

        # 2) continuous fire a beat later; the `fire` model auto-emits on spawn.
        time.sleep(FIRE_FUSE_SEC)
        if self._create(f'sdf_filename: "{FIRE_MODEL}", name: "{fire}", '
                        f'pose: {{position: {{x: {x} y: {y} z: {z}}}}}', fire):
            info(f"ignited {fire}")

    # --- idle re-arm (backup for when there's no detach signal) -----------
    def maybe_rearm_on_idle(self):
        # Only in fallback mode. In strict mode this would undo the disarm and let a
        # second contact from the same drop (bounce, roll, settle) book a second
        # record - and it is what previously masked the phantom pre-release drop
        # rather than preventing it.
        if self.require_detach:
            return
        if (not self.armed and self.last_contact_t
                and time.monotonic() - self.last_contact_t > REARM_IDLE_SEC):
            self.armed = True
            self.last_contact_t = 0.0
            info("re-armed: contacts idle, ready for next impact")


def enu_to_geodetic(x, y, origin):
    """World ENU metres -> (lat, lon), local-tangent-plane about the world origin.

    Uses the WGS84 meridional (M) and prime-vertical (N) radii rather than one
    spherical radius: at 47 deg latitude they differ by ~0.17%, which is 1.4 cm over
    8 m but 17 cm over 100 m - the wrong order of magnitude to ignore when the CEP
    being measured is sub-metre. Accurate to millimetres over a few km, which is the
    whole range a drop mission covers."""
    lat0 = math.radians(origin["lat"])
    s = math.sin(lat0)
    denom = 1.0 - WGS84_E2 * s * s
    m_rad = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)     # meridional
    n_rad = WGS84_A / math.sqrt(denom)                       # prime vertical
    return (origin["lat"] + math.degrees(y / m_rad),
            origin["lon"] + math.degrees(x / (n_rad * math.cos(lat0))))


def find_world_origin(world, explicit=None):
    """Geodetic origin of the world, for joining against a companion's lat/lon log.

    There is no gz service to read this back (only /world/<w>/set_spherical_coordinates),
    so it comes from the world SDF's <spherical_coordinates>. PX4_GZ_WORLDS is set by
    gz_env.sh when auto-started; the sibling worlds/ dir covers a by-hand run."""
    if explicit:
        return explicit

    cands = []
    if os.environ.get("PX4_GZ_WORLDS"):
        cands.append(os.path.join(os.environ["PX4_GZ_WORLDS"], f"{world}.sdf"))
    cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "worlds", f"{world}.sdf"))

    for path in cands:
        try:
            with open(path) as f:
                sdf = f.read()
        except OSError:
            continue
        block = re.search(r"<spherical_coordinates>(.*?)</spherical_coordinates>",
                          sdf, re.S)
        if not block:
            continue
        b = block.group(1)

        def field(tag, default=None):
            m = re.search(rf"<{tag}>\s*([^<\s]+)\s*</{tag}>", b)
            return m.group(1) if m else default

        try:
            lat, lon = float(field("latitude_deg")), float(field("longitude_deg"))
        except (TypeError, ValueError):
            continue
        frame = field("world_frame_orientation", "ENU")
        if frame != "ENU":
            # Every other axis convention would silently transpose/negate the derived
            # lat/lon. Refuse rather than emit plausible-looking wrong coordinates.
            warn(f"{world} uses world_frame_orientation {frame}, not ENU - "
                 "skipping geodetic origin")
            return None
        try:
            elev = float(field("elevation", "0"))
        except ValueError:
            elev = 0.0
        return {"lat": lat, "lon": lon, "elev": elev, "frame": frame,
                "source": os.path.basename(path)}
    return None


def topic_list():
    """`gz topic -l`, or [] if the daemon isn't answering."""
    try:
        out = subprocess.check_output(["gz", "topic", "-l"], text=True, timeout=10)
    except Exception:
        return []
    return out.splitlines()


def find_contact_topic(topics, model=None):
    """Pick the payload contact topic out of a topic list."""
    cands = [t for t in topics
             if t.endswith("/contact") and "/sensor/" in t and PAYLOAD_TAG in t]
    if not cands:  # fall back to any contact-sensor topic
        cands = [t for t in topics if t.endswith("/contact") and "/sensor/" in t]
    if model:  # with several instances up, take the one under our model
        scoped = [t for t in cands if f"/model/{model}/" in t]
        if scoped:
            return scoped[0]
    return cands[0] if cands else None


def find_world(topics):
    """World name from any /world/<name>/clock topic."""
    for t in topics:
        m = re.match(r"^/world/([^/]+)/clock$", t)
        if m:
            return m.group(1)
    return None


def world_is_alive(world):
    """True while the world's clock topic is still advertised. Stays true across a
    GUI pause (topics remain advertised even though nothing is published)."""
    return f"/world/{world}/clock" in topic_list()


def drone_model_from_topic(topic):
    """Carrier model instance from a contact topic path, e.g.
    /world/tank/model/x500_payload_0/model/payload/link/link/sensor/c/contact
    -> x500_payload_0."""
    m = re.match(r"^/world/[^/]+/model/([^/]+)/", topic)
    return m.group(1) if m else None


def wait_for_contact_topic(timeout, model=None):
    """Poll for the contact topic; the scorer is started right after the model is
    spawned, so the sensor may not be advertised yet."""
    deadline = time.monotonic() + timeout
    announced = False
    while True:
        topic = find_contact_topic(topic_list(), model)
        if topic:
            return topic
        if time.monotonic() >= deadline:
            return None
        if not announced:
            info(f"waiting up to {timeout:.0f}s for the payload contact topic")
            announced = True
        time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", help="contact topic (default: auto-detect)")
    ap.add_argument("--world", default=os.environ.get("PX4_GZ_WORLD"),
                    help="gz world name (default: $PX4_GZ_WORLD, else auto-detect)")
    ap.add_argument("--model",
                    help="carrier model instance, e.g. x500_payload_0 "
                         "(default: derived from the contact topic)")
    ap.add_argument("--explode", action="store_true",
                    help="spawn an explosion + fire at each HIT (needs model://fire resolvable)")
    ap.add_argument("--wait", type=float, default=0.0, metavar="SEC",
                    help="wait this long for the contact topic to appear (default: 0)")
    ap.add_argument("--exit-with-sim", action="store_true",
                    help="quit once the gz world is gone instead of idling forever")
    ap.add_argument("--detach-topic",
                    help="detach command topic; each message re-arms the scorer "
                         "(default: /model/<model>/detachable_joint/detach)")
    ap.add_argument("--target-pattern", default=TARGET_PATTERN, metavar="RE",
                    help=f"regex matching target model names (default: {TARGET_PATTERN!r}, "
                         "covers both 'm1-abrams' and 'abrams_0'/'abrams_1'/...)")
    ap.add_argument("--record", metavar="PATH",
                    help="append one JSON record per drop here "
                         "(default: payload_impacts_<carrier>.jsonl; '' disables)")
    ap.add_argument("--run-id", help="run identifier stamped into each record "
                                     "(default: UTC start time)")
    ap.add_argument("--aim-model", metavar="NAME",
                    help="intended target model, for ballistics-only runs. Leave unset "
                         "with a tracker in the loop and join its aimpoint log offline "
                         "- the system under test must not grade itself.")
    ap.add_argument("--aimpoint", metavar="X,Y",
                    help="intended impact point in world coordinates (overrides --aim-model)")
    ap.add_argument("--origin", metavar="LAT,LON[,ELEV]",
                    help="world geodetic origin, for deriving lat/lon in the records "
                         "(default: read <spherical_coordinates> from the world SDF)")
    args = ap.parse_args()

    origin = None
    if args.origin:
        try:
            parts = [float(v) for v in args.origin.split(",")]
            origin = {"lat": parts[0], "lon": parts[1],
                      "elev": parts[2] if len(parts) > 2 else 0.0,
                      "frame": "ENU", "source": "cli"}
        except (ValueError, IndexError):
            err(f"--origin expects 'LAT,LON[,ELEV]', got {args.origin!r}")
            return 1

    aim_xy = None
    if args.aimpoint:
        try:
            ax, ay = (float(v) for v in args.aimpoint.split(","))
            aim_xy = (ax, ay)
        except ValueError:
            err(f"--aimpoint expects 'X,Y', got {args.aimpoint!r}")
            return 1

    install_signal_handlers()

    topics = topic_list()
    world = args.world or find_world(topics) or DEFAULT_WORLD

    topic = args.topic or find_contact_topic(topics, args.model)
    if not topic and args.wait > 0:
        topic = wait_for_contact_topic(args.wait, args.model)
    if not topic:
        err("no contact topic found, is the sim running with the contact sensor loaded? "
            "(gz topic -l | grep contact)")
        return 1

    drone_model = args.model or drone_model_from_topic(topic)
    detach_topic = (args.detach_topic or
                    (f"/model/{drone_model}/detachable_joint/detach" if drone_model else None))
    info(f"world: {world}, carrier: {drone_model or 'unknown'}")
    info(f"listening on {topic}")

    # --record '' disables; unset picks a per-carrier default in the rootfs cwd,
    # next to the scorer's own .log that px4-rc.gzsim tees.
    record_path = args.record
    if record_path is None:
        record_path = f"payload_impacts_{drone_model or 'unknown'}.jsonl"
    run_id = args.run_id or time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())

    origin = find_world_origin(world, origin)
    if origin:
        info(f"world origin {origin['lat']:.9f}, {origin['lon']:.9f} "
             f"({origin['frame']}, from {origin['source']})")
    else:
        warn(f"no geodetic origin for world '{world}' - records carry world metres "
             "only, so joining them to a companion's lat/lon log needs the transform "
             "supplied by hand (--origin LAT,LON)")

    node = Node()
    scorer = Scorer(world, drone_model, explode=args.explode,
                    target_pattern=args.target_pattern,
                    record_path=record_path or None, run_id=run_id,
                    aim_model=args.aim_model, aim_xy=aim_xy, origin=origin)

    if not node.subscribe(contacts_pb2.Contacts, topic, scorer.on_contact):
        err(f"failed to subscribe to {topic}")
        return 1
    # Track every target's truth pose for miss vectors (static targets appear here too).
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", scorer.on_pose)
    # Arm on each new drop command. Whether this works decides the arming policy, so
    # unlike before it is not silently ignored - a failure downgrades to fallback mode
    # loudly instead of leaving the scorer permanently disarmed.
    detach_ok = False
    if detach_topic:
        try:
            detach_ok = node.subscribe(empty_pb2.Empty, detach_topic, scorer.on_detach)
        except Exception as e:
            warn(f"could not subscribe to {detach_topic}: {e}")
        if not detach_ok:
            warn(f"detach topic {detach_topic} not subscribed")
        elif detach_topic not in topic_list():
            # subscribe() succeeds for a topic nobody publishes, so a True return does
            # NOT prove the detach signal will arrive. Strict arming makes that
            # load-bearing: a wrong or absent topic would mean zero records rather
            # than noisy ones. gz_bridge advertises this topic and the DetachableJoint
            # plugin subscribes to it, so either end makes it appear in the list; if it
            # is missing, something is genuinely wrong and permissive is the safer mode.
            warn(f"detach topic {detach_topic} is not advertised by anyone - "
                 "is the DetachableJoint plugin in the model?")
            detach_ok = False
    scorer.set_detach_mode(detach_ok)

    info(f"scoring impacts ({'explosion VFX on' if args.explode else 'score only'})")
    if record_path:
        info(f"run {run_id}, appending drop records to {record_path}")
    if args.aim_model or aim_xy:
        info(f"aimpoint declared: {args.aimpoint or args.aim_model}")
    next_sim_poll = time.monotonic() + SIM_POLL_SEC
    sim_misses = 0
    try:
        while True:
            time.sleep(0.2)
            scorer.maybe_rearm_on_idle()

            if args.exit_with_sim and time.monotonic() >= next_sim_poll:
                next_sim_poll = time.monotonic() + SIM_POLL_SEC
                sim_misses = 0 if world_is_alive(world) else sim_misses + 1
                if sim_misses >= SIM_MISSES_TO_EXIT:
                    info(f"world '{world}' is gone, exiting")
                    return 0
    except KeyboardInterrupt:
        info("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
