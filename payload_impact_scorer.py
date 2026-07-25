#!/usr/bin/env python3
"""
payload_impact_scorer.py — score a dropped payload against the M1 Abrams in the
`tank` PX4 SITL world by listening to the payload's contact sensor.
Can expand to different collisions later

Piece 2 of the drop-accuracy pipeline: subscribe to the contact stream, filter by
collision name, debounce to one event per drop, and print HIT/MISS + miss-distance.
Piece 3 (--explode): on a HIT, set off an explosion burst and then a lingering fire
at the impact point.

Normally you do NOT run this by hand: px4-rc.gzsim starts it in the background for
any `*payload*` airframe (see Tools/simulation/payload_dropper_docs.md). It is
launched right after the model is spawned, so it waits for the contact topic to
appear (--wait) instead of giving up, and it exits when the sim goes away
(--exit-with-sim) so it does not outlive the world it was scoring.

VFX approach — auto-emit on spawn, NOT topic triggering:
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
    python3 payload_impact_scorer.py --world tank --model x500_payload_0 \
        --wait 60 --exit-with-sim --explode                # how px4-rc.gzsim calls it
"""
import os
# MUST be set before any gz.msgs (protobuf) import on this system.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import argparse
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
# Configuration — match the scoped names seen in `gz topic -e` on the contact topic.
# ---------------------------------------------------------------------------
DEFAULT_WORLD = "tank"
PAYLOAD_TAG = "payload"          # substring identifying the payload side of a contact
TARGETS = {                      # scoped-name suffix -> label
    "hull_collision":   "hull",
    "turret_collision": "turret",
}
GROUND_TAG = "ground_plane"
TANK_MODEL = "m1-abrams"         # used to auto-track the tank position for miss-distance

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
    has `sh` set SIGINT/SIGQUIT to SIG_IGN for async jobs — a disposition that
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
            pass  # not on the main thread / unsupported — the poll backstop still applies


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
    def __init__(self, world, drone_model, explode=False):
        self.world = world
        # Contacts whose *other* side matches the carrier are self-contacts (payload
        # releasing from / brushing the drone) and are skipped without disarming, so
        # the first real impact against the tank or ground is what gets scored.
        self.ignore_tags = [drone_model] if drone_model else []
        self.explode = explode
        self.armed = True            # ready to score the next impact
        self.last_contact_t = 0.0
        self.tank_xy = None          # (x, y) once we hear a pose for TANK_MODEL
        self.hit_count = 0

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

    def classify(self, other):
        for suffix, label in TARGETS.items():
            if other.endswith(suffix):
                return "HIT", label
        if GROUND_TAG in other:
            return "MISS", "ground"
        return "OTHER", other

    # --- callbacks --------------------------------------------------------
    def on_pose(self, msg: pose_v_pb2.Pose_V):
        for p in msg.pose:
            if p.name == TANK_MODEL:
                self.tank_xy = (p.position.x, p.position.y)

    def on_detach(self, _msg: empty_pb2.Empty):
        # A new drop was commanded -> arm for the next impact.
        self.armed = True
        info("re-armed: detach commanded, waiting for impact")

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
            return  # only self-contacts this message — stay armed

        other = self._other(c)
        kind, label = self.classify(other)
        x, y, z = self._centroid(c)
        force = self._peak_force_z(c)

        self.armed = False  # debounce: one event per drop until re-armed

        if kind == "HIT":
            self.hit_count += 1
            info(f"HIT {label} at ({x:+.2f}, {y:+.2f}, {z:+.2f}), "
                 f"peak Fz {force:.1f} N, against {other}")
            if self.explode:
                # Run off-thread: the effect sleeps for the fuse and must not
                # block the transport callback.
                threading.Thread(target=self.detonate_and_ignite,
                                 args=(x, y, z), daemon=True).start()
        elif kind == "MISS":
            dist = self._miss_distance(x, y)
            dstr = f"{dist:.2f} m from tank" if dist is not None else "tank pos unknown"
            info(f"MISS ground at ({x:+.2f}, {y:+.2f}), {dstr}")
        else:
            info(f"contact with {other} at ({x:+.2f}, {y:+.2f}, {z:+.2f})")

    # --- scoring / actions ------------------------------------------------
    def _miss_distance(self, x, y):
        if self.tank_xy is None:
            return None
        return math.hypot(x - self.tank_xy[0], y - self.tank_xy[1])

    def _create(self, req, what):
        """Call the world's factory service to spawn something. The CLI echoes the
        Boolean reply ("data: true") on stdout — capture it instead of letting it
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
        particle_emitter command topic is used — works the same on gz-sim 8.11 & 8.14.
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
        if (not self.armed and self.last_contact_t
                and time.monotonic() - self.last_contact_t > REARM_IDLE_SEC):
            self.armed = True
            self.last_contact_t = 0.0
            info("re-armed: contacts idle, ready for next impact")


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
    args = ap.parse_args()

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

    node = Node()
    scorer = Scorer(world, drone_model, explode=args.explode)

    if not node.subscribe(contacts_pb2.Contacts, topic, scorer.on_contact):
        err(f"failed to subscribe to {topic}")
        return 1
    # Track tank position for miss-distance (static tank still appears here).
    node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", scorer.on_pose)
    # Re-arm on each new drop command (best-effort; ignore if type/topic differs).
    if detach_topic:
        try:
            node.subscribe(empty_pb2.Empty, detach_topic, scorer.on_detach)
        except Exception:
            pass

    info(f"armed, scoring impacts ({'explosion VFX on' if args.explode else 'score only'})")
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
