"""
CDS215 — ESCAPE THE HUNTER: SAFE HOUSE (Student Starter)

Classroom starter: original game works; a short code-along activates the safe house.
Requires: pip install pybullet
Run: python Hunter_Recovery_Fix_Student.py
Controls: W/Up = forward, S/Down = reverse, A/Left = steer left,
          D/Right = steer right, R = restart, Q = quit.

Optional instructor test (no GUI): python Hunter_Recovery_Fix_Student.py --smoke-test
The smoke test parks the player and verifies that the hunter eventually catches it.

HOW THIS CONNECTS TO WEEK 10:
 - Husky URDF and wheel joint IDs, just as in our earlier program.
 - Student teleoperation controls motor speeds.
 - Hunter continuously uses the student's position as a MOVING WAYPOINT.
 - PyBullet rayTest() helps the hunter turn around obstacles.
 - Contact detection decides whether the hunter caught the student.
 - Physics, graphics, and playback settings preserve our fast-simulation work.

IMPORTANT: Physics rate and drawing rate do NOT directly control motor speed.
MAX_PLAYBACK_SPEED changes how quickly simulated time is displayed.
"""

import math
import sys
import time
import pybullet as p
import pybullet_data


# =====================================================================
# PART 1 — GAME SETTINGS: EASY FOR STUDENTS TO EXPERIMENT WITH
# =====================================================================
SURVIVAL_SECONDS = 60            # Stay uncaught this many SIMULATED seconds.
PLAYER_SPEED = 6              # Student Husky forward motor speed.
PLAYER_TURN_SPEED = 6          # Student Husky turning rate setting.
HUNTER_SPEED = 14              # Hunter motor speed.
HUNTER_TURN_SPEED = 5.5          # Hunter turn speed
HUNTER_START = (5.8, 5.5)
PLAYER_START = (-5.7, -5.5)

# SAFE HOUSE ADD-ON — a timed protected zone, not an unlimited hiding place.
# Both the GREEN marker and the game rule are supplied for the class.
SAFE_HOUSE_X = -6.1
SAFE_HOUSE_Y = 4.7
SAFE_HOUSE_RADIUS = 1.15          # Distance from center to activate the zone (meters).
SAFE_HOUSE_SECONDS = 12.0          # Total simulated seconds of protection per game.

# These settings affect PERFORMANCE and PLAYBACK, not just wheel speeds.
PHYSICS_HZ = 120                 # Try 120 or 240. More Hz = more physics work.
RAY_DRAW_HZ = 10                 # Try 2, 10, 30, 120: how often rays are drawn.
MAX_PLAYBACK_SPEED = 1.0        # 1.0 = normal; 1.5/2.0 = faster; 0 = unlimited.
SHOW_HUNTER_RAYS = True          # Turn off if graphics are slow.
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 750

# Instructor tuning: deliberately separate from the student settings.
ARENA_HALF_SIZE = 9.0
WALL_THICKNESS = 0.6
HUNTER_SENSOR_RANGE = 2.6
# Three forward-looking rays: center plus left and right FRONT CORNERS.
# Wide-angle rays used to miss obstacles directly in front of a corner.
HUNTER_SENSOR_ANGLE = math.radians(12)  # Slight outward sweep, not 35 degrees.
HUNTER_STOP_DISTANCE = 1.5
HUNTER_SIDE_DISTANCE = 1.15           # Trigger when EITHER corner gets too close.
HUNTER_SENSOR_OFFSET = 0.72         # Forward of the chassis to reduce self-hits.
HUNTER_CORNER_OFFSET = 0.36         # Left/right position measured from centerline.

# A single avoidance TURN is not enough: without forward follow-through the
# hunter immediately aims at the player and swings back toward the same box.
HUNTER_AVOID_TURN_SECONDS = 0.55     # Rotate away from the detected box.
HUNTER_ADVANCE_SECONDS = 1.85         # Then drive past its corner before chasing.
HUNTER_ADVANCE_SPEED_FACTOR = 0.65   # Moderate, controlled forward motion.
HUNTER_ADVANCE_FRONT_CLEARANCE = 0.70  # Emergency stop/turn if box is too close.
HUNTER_ADVANCE_SIDE_CLEARANCE = 0.35   # Keep the chassis corners safe.

# STUCK RECOVERY — applies during chase AND obstacle-avoidance sequences.
HUNTER_STUCK_CHECK_SECONDS = 2.0    # How often to check actual forward progress.
HUNTER_STUCK_MIN_MOVEMENT = 0.30    # Less movement means physically trapped.
HUNTER_REVERSE_SECONDS = 0.95       # Back away from the obstacle.
HUNTER_RECOVERY_SPIN_SECONDS = 1.10 # Turn in place after reversing.
HUNTER_RECOVERY_ADVANCE_SECONDS = 1.5  # Move away before resuming pursuit.
SENSOR_HEIGHT = 0.38
LEFT_WHEELS = (2, 4)
RIGHT_WHEELS = (3, 5)

# Each item: center x, center y, FULL width, FULL depth, RGB color.
# The course deliberately provides space to weave around the obstacles.
OBSTACLES = [
    (-3.0,  1.2, 2.6, 1.2, (0.15, 0.75, 0.95)),
    ( 1.1, -2.1, 3.0, 1.25, (0.95, 0.62, 0.15)),
    ( 3.5,  2.8, 1.3, 2.8, (0.68, 0.34, 0.91)),
    (-4.1, -3.3, 1.6, 1.5, (0.94, 0.74, 0.18)),
    (-0.6,  4.1, 2.5, 1.2, (0.40, 0.88, 0.54)),
    ( 4.3, -4.1, 1.7, 1.7, (0.91, 0.41, 0.61)),
]

SMOKE_TEST = "--smoke-test" in sys.argv
if not isinstance(PHYSICS_HZ, int) or PHYSICS_HZ < 30:
    raise ValueError("PHYSICS_HZ must be an integer of at least 30")
if not isinstance(RAY_DRAW_HZ, int) or RAY_DRAW_HZ < 1:
    raise ValueError("RAY_DRAW_HZ must be an integer of at least 1")
if MAX_PLAYBACK_SPEED < 0:
    raise ValueError("MAX_PLAYBACK_SPEED cannot be negative")
if SURVIVAL_SECONDS <= 0:
    raise ValueError("SURVIVAL_SECONDS must be positive")


# =====================================================================
# PART 2 — REUSABLE ROBOT COMMANDS (FROM OUR PREVIOUS HUSKY LABS)
# =====================================================================
_last_commands = {}


def set_wheels(robot_id, left_speed, right_speed):
    """Only send a command if it changes (our motor optimization)."""
    speeds = (round(left_speed, 4), round(right_speed, 4))
    if _last_commands.get(robot_id) == speeds:
        return
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_speed, force=100)
    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_speed, force=100)
    _last_commands[robot_id] = speeds


def stop_robot(robot_id):
    set_wheels(robot_id, 0, 0)


def robot_xy_heading(robot_id):
    position, rotation = p.getBasePositionAndOrientation(robot_id)
    return position[0], position[1], p.getEulerFromQuaternion(rotation)[2]


def angle_difference(target_angle, current_angle):
    return math.atan2(math.sin(target_angle - current_angle),
                      math.cos(target_angle - current_angle))


def pressed(keys, *codes):
    return any(keys.get(code, 0) & p.KEY_IS_DOWN for code in codes)


def player_teleop(robot_id, keys):
    """WASD / arrow controls; holding forward and turn at once makes an arc."""
    up = getattr(p, "B3G_UP_ARROW", -100)
    down = getattr(p, "B3G_DOWN_ARROW", -101)
    left_arrow = getattr(p, "B3G_LEFT_ARROW", -102)
    right_arrow = getattr(p, "B3G_RIGHT_ARROW", -103)

    forward = pressed(keys, ord('w'), ord('W'), up)
    backward = pressed(keys, ord('s'), ord('S'), down)
    left = pressed(keys, ord('a'), ord('A'), left_arrow)
    right = pressed(keys, ord('d'), ord('D'), right_arrow)

    drive = (int(forward) - int(backward)) * PLAYER_SPEED
    turn = (int(left) - int(right)) * PLAYER_TURN_SPEED
    set_wheels(robot_id, drive - turn, drive + turn)


# =====================================================================
# PART 3 — ROBOT COLORS, ARENA, AND SCENE SETUP
# =====================================================================
def make_box(x, y, width, depth, color, height=1.2):
    half = [width / 2, depth / 2, height / 2]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                 rgbaColor=[*color, 1])
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, height / 2])


def tint_husky(robot_id, rgb):
    """Give the two otherwise-identical Husky models distinct team colors."""
    for link in [-1] + list(range(p.getNumJoints(robot_id))):
        try:
            p.changeVisualShape(robot_id, link, rgbaColor=[*rgb, 1])
        except p.error:
            pass  # Some URDF links have no visible shape.


def create_arena():
    size = ARENA_HALF_SIZE
    thick = WALL_THICKNESS
    blue_gray = (0.28, 0.35, 0.48)
    make_box(0, size, size * 2 + thick, thick, blue_gray)
    make_box(0, -size, size * 2 + thick, thick, blue_gray)
    make_box(size, 0, thick, size * 2 + thick, blue_gray)
    make_box(-size, 0, thick, size * 2 + thick, blue_gray)
    for x, y, w, d, color in OBSTACLES:
        make_box(x, y, w, d, color)


def create_safe_house():
    """Draw a green floor marker. The marker does NOT block or collide with robots."""
    marker = p.createVisualShape(
        p.GEOM_CYLINDER, radius=SAFE_HOUSE_RADIUS, length=0.04,
        rgbaColor=[0.12, 0.94, 0.30, 0.65])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker,
                      basePosition=[SAFE_HOUSE_X, SAFE_HOUSE_Y, 0.025])
    if not SMOKE_TEST:
        p.addUserDebugText("SAFE HOUSE (6s TOTAL)",
                           [SAFE_HOUSE_X - 1.4, SAFE_HOUSE_Y + 0.45, 0.8],
                           textColorRGB=[0.05, 0.60, 0.12], textSize=1.05)


def make_husky(start, rgb):
    rid = p.loadURDF("husky/husky.urdf", [start[0], start[1], 0.22])
    tint_husky(rid, rgb)
    return rid


# =====================================================================
# PART 4 — THE HUNTER: THE PLAYER IS ITS MOVING WAYPOINT
# =====================================================================
# Students do not have to write the hunting algorithm. Instructor can
# explain just these two lines from update_hunter():
#     player_x, player_y, _ = robot_xy_heading(player_id)
#     target_angle = math.atan2(player_y - hunter_y, player_x - hunter_x)
# The target changes EVERY step, so the hunter always knows the location.

_ray_ids = {"center": -1, "left": -1, "right": -1}


def hunter_sensor(hunter_id, player_id, x, y, heading, angle, step):
    """Return the distance to the nearest wall/box using a front-facing ray.

    The center ray originates at the nose. Left/right rays originate near
    the front corners and fan slightly outward to protect the full width.
    The player is not an obstacle to avoid because catching it is the goal.
    Detection runs EVERY physics step; visuals refresh less frequently.
    """
    # The center ray starts at the nose. The two side rays start at the
    # FRONT CORNERS of the Husky instead of on the 35-degree diagonal.
    # "side" is signed: + means left, - means right, 0 means center.
    side = HUNTER_CORNER_OFFSET if angle > 0 else (
        -HUNTER_CORNER_OFFSET if angle < 0 else 0.0
    )
    start = [x + HUNTER_SENSOR_OFFSET * math.cos(heading) - side * math.sin(heading),
             y + HUNTER_SENSOR_OFFSET * math.sin(heading) + side * math.cos(heading),
             SENSOR_HEIGHT]
    direction = heading + angle
    end = [start[0] + HUNTER_SENSOR_RANGE * math.cos(direction),
           start[1] + HUNTER_SENSOR_RANGE * math.sin(direction), SENSOR_HEIGHT]
    hit_id, _, fraction, _, _ = p.rayTest(start, end)[0]
    # Treat the player as clear so the enemy runs into the player to win.
    is_wall = hit_id >= 0 and hit_id not in (hunter_id, player_id)
    distance = max(0.0, fraction * HUNTER_SENSOR_RANGE) if is_wall else HUNTER_SENSOR_RANGE

    draw_interval = max(1, round(PHYSICS_HZ / RAY_DRAW_HZ))
    if SHOW_HUNTER_RAYS and not SMOKE_TEST and step % draw_interval == 0:
        label = "center" if angle == 0 else ("left" if angle > 0 else "right")
        rgb = [1, 0.15, 0.1] if is_wall else [0.1, 0.9, 0.2]
        _ray_ids[label] = p.addUserDebugLine(
            start, end, rgb, lineWidth=2,
            replaceItemUniqueId=_ray_ids[label])
    return distance


def update_hunter(hunter_id, player_id, game):
    """Always pursue player's LIVE position; avoid boxes and recover if stuck."""
    hunter_x, hunter_y, hunter_heading = robot_xy_heading(hunter_id)
    player_x, player_y, _ = robot_xy_heading(player_id)  # MOVING WAYPOINT!
    target_angle = math.atan2(player_y - hunter_y, player_x - hunter_x)
    error = angle_difference(target_angle, hunter_heading)
    step = game["step"]

    # Detect a physically trapped hunter, including when it switches between
    # AVOID and ADVANCE without actually moving around the box. Previously
    # this check only ran during CHASE, so that loop could last indefinitely.
    if step - game["anchor_step"] >= round(HUNTER_STUCK_CHECK_SECONDS * PHYSICS_HZ):
        anchor_x, anchor_y = game["anchor"]
        moved = math.hypot(hunter_x - anchor_x, hunter_y - anchor_y)
        to_player = math.hypot(player_x - hunter_x, player_y - hunter_y)
        if (moved < HUNTER_STUCK_MIN_MOVEMENT
                and to_player > 0.75
                and game["mode"] in ("chase", "avoid", "advance")):
            old_mode = game["mode"]
            game["mode"] = "reverse"
            game["mode_until"] = step + round(HUNTER_REVERSE_SECONDS * PHYSICS_HZ)
            game["recover_direction"] *= -1  # Alternate to escape dead ends.
            if not SMOKE_TEST:
                print(f"HUNTER STUCK in {old_mode.upper()}: moved {moved:.2f} m "
                      f"in {HUNTER_STUCK_CHECK_SECONDS:.1f}s. Reversing and turning.")
        game["anchor"] = (hunter_x, hunter_y)
        game["anchor_step"] = step

    if game["mode"] == "reverse":
        if step < game["mode_until"]:
            set_wheels(hunter_id, -HUNTER_SPEED * 0.6, -HUNTER_SPEED * 0.6)
            return
        game["mode"] = "spin"
        game["mode_until"] = step + round(HUNTER_RECOVERY_SPIN_SECONDS * PHYSICS_HZ)

    if game["mode"] == "spin":
        if step < game["mode_until"]:
            d = game["recover_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do not point straight at the player again after getting unstuck;
        # commit to moving away from the box before returning to CHASE.
        game["mode"] = "advance"
        game["mode_until"] = step + round(HUNTER_RECOVERY_ADVANCE_SECONDS * PHYSICS_HZ)
        game["avoid_direction"] = game["recover_direction"]
        game["anchor"] = (hunter_x, hunter_y)
        game["anchor_step"] = step

    if game["mode"] == "avoid":
        if step < game["mode_until"]:
            d = game["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do NOT immediately aim back at the player. First get past the box.
        game["mode"] = "advance"
        game["mode_until"] = step + round(HUNTER_ADVANCE_SECONDS * PHYSICS_HZ)

    # Sensors monitor the path every step. Drawing the rays is optional.
    front = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, 0, step)
    left = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                         hunter_heading, HUNTER_SENSOR_ANGLE, step)
    right = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, -HUNTER_SENSOR_ANGLE, step)

    # Finish the maneuver with a committed forward move. We intentionally
    # ignore the usual EARLY-warning distances during this short stage: they
    # may still see the box alongside the Husky. But all three rays remain
    # active, and emergency clearances can interrupt forward travel.
    if game["mode"] == "advance":
        too_close = (front < HUNTER_ADVANCE_FRONT_CLEARANCE
                     or left < HUNTER_ADVANCE_SIDE_CLEARANCE
                     or right < HUNTER_ADVANCE_SIDE_CLEARANCE)
        if too_close:
            game["mode"] = "avoid"
            # Maintain the chosen detour direction rather than oscillating.
            game["mode_until"] = step + round(0.32 * PHYSICS_HZ)
            d = game["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        if step < game["mode_until"]:
            speed = HUNTER_SPEED * HUNTER_ADVANCE_SPEED_FACTOR
            set_wheels(hunter_id, speed, speed)
            return
        game["mode"] = "chase"

    # During the normal chase, ANY ray can trigger a full avoidance maneuver.
    if (front < HUNTER_STOP_DISTANCE
            or left < HUNTER_SIDE_DISTANCE
            or right < HUNTER_SIDE_DISTANCE):
        game["mode"] = "avoid"
        game["avoid_direction"] = 1 if left >= right else -1
        game["mode_until"] = step + round(HUNTER_AVOID_TURN_SECONDS * PHYSICS_HZ)
        d = game["avoid_direction"] * HUNTER_TURN_SPEED
        set_wheels(hunter_id, -d, d)
        return

    # Pure pursuit: steer toward the player's current x/y coordinates.
    # Slower turning creates opportunities for the student to outmaneuver it.
    if abs(error) > 0.72:
        turn = HUNTER_TURN_SPEED if error > 0 else -HUNTER_TURN_SPEED
        set_wheels(hunter_id, -turn, turn)
    else:
        turn = max(-HUNTER_TURN_SPEED, min(HUNTER_TURN_SPEED, error * 5.0))
        drive = HUNTER_SPEED * (0.78 if abs(error) > 0.38 else 1.0)
        set_wheels(hunter_id, drive - turn, drive + turn)


# =====================================================================
# PART 5 — GAME RULES: COUNTDOWN, CONTACT, WIN, LOSS, AND RESTART
# =====================================================================
_hud_id = -1
_safe_hud_id = -1
_mode_hud_id = -1


def update_safe_house(player_id, game):
    """Check whether the player has protection remaining inside the green marker."""
    px, py, _ = robot_xy_heading(player_id)
    distance_to_house = math.hypot(px - SAFE_HOUSE_X, py - SAFE_HOUSE_Y)
    inside_house = distance_to_house < SAFE_HOUSE_RADIUS

    # ===========================================================
    # STUDENT CODE-ALONG (ABOUT 3 LINES OF NEW PYTHON)
    # Change the next line so protection activates ONLY when BOTH:
    #   1) the player is inside_house, AND
    #   2) safe_steps_used is less than SAFE_HOUSE_SECONDS * PHYSICS_HZ.
    # Then add an if statement to count one step when protected.
    # ===========================================================
    game["safe_active"] = (
                            inside_house and game["safe_steps_used"] < SAFE_HOUSE_SECONDS * PHYSICS_HZ
    )
      # REPLACE ME
    # STUDENT: if safe_active, add 1 to game["safe_steps_used"].
    if game["safe_active"]:
        game["safe_steps_used"] += 1

def status_hud(game, player_id, hunter_id, force=False):
    """One reused debug label every half-second, not every physics frame."""
    global _hud_id, _safe_hud_id, _mode_hud_id
    if SMOKE_TEST:
        return
    update_period = max(1, PHYSICS_HZ // 2)
    if not force and game["step"] % update_period != 0:
        return
    px, py, _ = robot_xy_heading(player_id)
    hx, hy, _ = robot_xy_heading(hunter_id)
    separation = math.hypot(px - hx, py - hy)
    remaining = max(0, SURVIVAL_SECONDS - game["step"] / PHYSICS_HZ)
    if game["result"] == "PLAYING":
        label = f"ESCAPE THE HUNTER | {remaining:05.1f}s | HUNTER {separation:.1f}m AWAY"
        color = [1, 0.97, 0.25]
    elif game["result"] == "WIN":
        label = "YOU WIN!  Survived!    R = PLAY AGAIN    Q = QUIT"
        color = [0.2, 1, 0.4]
    else:
        label = "CAUGHT!  HUNTER WINS.    R = RETRY    Q = QUIT"
        color = [1, 0.3, 0.3]
    _hud_id = p.addUserDebugText(label, [-8.1, 8.05, 1.7],
                                 textColorRGB=color, textSize=1.25,
                                 replaceItemUniqueId=_hud_id)

    # Keep the safe-house status separate so the main HUD stays readable.
    safe_left = max(0.0, SAFE_HOUSE_SECONDS - game["safe_steps_used"] / PHYSICS_HZ)
    if game["safe_active"]:
        safe_label = f"SAFE HOUSE ACTIVE | {safe_left:.1f}s protection left"
        safe_color = [0.20, 1.0, 0.30]
    elif safe_left > 0:
        safe_label = f"SAFE HOUSE READY | {safe_left:.1f}s remaining"
        safe_color = [0.95, 0.91, 0.25]
    else:
        safe_label = "SAFE HOUSE EXHAUSTED — KEEP MOVING!"
        safe_color = [1.0, 0.45, 0.2]
    _safe_hud_id = p.addUserDebugText(
        safe_label, [-8.1, 7.35, 1.7], textColorRGB=safe_color,
        textSize=1.1, replaceItemUniqueId=_safe_hud_id)

    # A short diagnostic label helps distinguish "stuck" from "caught".
    _mode_hud_id = p.addUserDebugText(
        f"HUNTER MODE: {game['mode'].upper()}", [-8.1, 6.65, 1.7],
        textColorRGB=[0.8, 0.94, 1.0], textSize=1.0,
        replaceItemUniqueId=_mode_hud_id)


def reset_game():
    """Create a fresh arena and reset all counters (R key)."""
    global _hud_id, _safe_hud_id, _mode_hud_id
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / PHYSICS_HZ)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")
    create_arena()
    create_safe_house()
    player_id = make_husky(PLAYER_START, (0.16, 0.49, 1.0))
    hunter_id = make_husky(HUNTER_START, (1.0, 0.16, 0.18))
    p.resetDebugVisualizerCamera(cameraDistance=24, cameraYaw=0,
                                  cameraPitch=-88, cameraTargetPosition=[0, 0, 0])
    _last_commands.clear()
    for key in _ray_ids:
        _ray_ids[key] = -1
    _hud_id = -1
    _safe_hud_id = -1
    _mode_hud_id = -1
    game = {
        "step": 0,
        "safe_active": False,
        "safe_steps_used": 0,
        "result": "PLAYING",
        "mode": "chase",
        "mode_until": 0,
        "avoid_direction": 1,
        "recover_direction": 1,
        "anchor": HUNTER_START,
        "anchor_step": 0,
        "wall_start": time.monotonic(),
        "report_start": time.monotonic(),
        "report_step": 0,
    }
    status_hud(game, player_id, hunter_id, force=True)
    print("\nESCAPE THE HUNTER — NEW GAME")
    print("Blue = YOU (WASD / arrows), Red = HUNTER.  Survive for", SURVIVAL_SECONDS, "seconds.")
    print("Drive onto the green SAFE HOUSE for up to 6 simulated seconds of protection.")
    print("R = restart | Q = quit | Click the PyBullet window for keyboard focus.")
    print(f"Physics {PHYSICS_HZ} Hz | sensor drawing {RAY_DRAW_HZ} Hz "
          f"| playback cap {MAX_PLAYBACK_SPEED or 'unlimited'}x")
    return player_id, hunter_id, game


def run():
    if SMOKE_TEST:
        client = p.connect(p.DIRECT)
    else:
        client = p.connect(p.GUI, options=f"--width={WINDOW_WIDTH} --height={WINDOW_HEIGHT}")
    if client < 0:
        raise RuntimeError("Could not start PyBullet. Install pybullet and use a desktop GUI.")

    if not SMOKE_TEST:
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    player_id, hunter_id, game = reset_game()
    print("Hunter GPS target updated every step. The green SAFE HOUSE provides limited protection.")

    try:
        while p.isConnected():
            keys = {} if SMOKE_TEST else p.getKeyboardEvents()
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('q'), ord('Q'))):
                break
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('r'), ord('R'))):
                player_id, hunter_id, game = reset_game()
                continue

            if game["result"] != "PLAYING":
                if SMOKE_TEST:
                    break
                time.sleep(0.04)  # No physics work once win/loss has been decided.
                continue

            if SMOKE_TEST:
                stop_robot(player_id)  # Player stays parked: hunter should catch it.
            else:
                player_teleop(player_id, keys)

            # Prewritten game mechanic: activate the safe house when nearby.
            was_safe = game["safe_active"]
            update_safe_house(player_id, game)
            if game["safe_active"] and not was_safe and not SMOKE_TEST:
                print("SAFE HOUSE ACTIVE! Hunter freezes while protection remains.")
            elif was_safe and not game["safe_active"] and not SMOKE_TEST:
                print("SAFE HOUSE ENDED. The hunter is pursuing again!")

            # The hunter never forgets the player's coordinates. The safe
            # house simply gives the student a brief, limited respite.
            if game["safe_active"]:
                stop_robot(hunter_id)
                hunter_x, hunter_y, _ = robot_xy_heading(hunter_id)
                game["anchor"] = (hunter_x, hunter_y)
                game["anchor_step"] = game["step"]
            else:
                update_hunter(hunter_id, player_id, game)

            p.stepSimulation()
            game["step"] += 1
            elapsed = game["step"] / PHYSICS_HZ

            # Real rigid-body collision, not center-point distance alone.
            if (p.getContactPoints(bodyA=player_id, bodyB=hunter_id)
                    and not game["safe_active"]):
                game["result"] = "LOSS"
                print(f"CAUGHT! Hunter touched player after {elapsed:.1f} simulated seconds.")
            elif elapsed >= SURVIVAL_SECONDS:
                game["result"] = "WIN"
                print(f"YOU WIN! Survived {SURVIVAL_SECONDS} simulated seconds.")

            if game["result"] != "PLAYING":
                stop_robot(player_id)
                stop_robot(hunter_id)
                status_hud(game, player_id, hunter_id, force=True)
                if SMOKE_TEST:
                    break

            status_hud(game, player_id, hunter_id)

            # Instructor may compare real-time performance on different PCs.
            now = time.monotonic()
            if not SMOKE_TEST and now - game["report_start"] >= 5.0:
                sim_time = (game["step"] - game["report_step"]) / PHYSICS_HZ
                speed = sim_time / (now - game["report_start"])
                print(f"SIMULATION SPEED: {speed:.2f}x real time "
                      f"| {max(0.0, SURVIVAL_SECONDS - elapsed):.1f}s left")
                game["report_start"] = now
                game["report_step"] = game["step"]

            # A single short delay per BLOCK, not 120 tiny sleeps per second.
            # 0 means run without a cap; GUI may be too quick for teleop.
            if not SMOKE_TEST and MAX_PLAYBACK_SPEED > 0:
                block = max(1, PHYSICS_HZ // 20)
                if game["step"] % block == 0:
                    desired = game["step"] / (PHYSICS_HZ * MAX_PLAYBACK_SPEED)
                    wait = desired - (now - game["wall_start"])
                    if wait > 0:
                        time.sleep(wait)

            if SMOKE_TEST and elapsed >= 90.0:
                break

        if SMOKE_TEST:
            if game["result"] == "LOSS":
                print(f"SMOKE TEST PASS: stationary player caught in {game['step'] / PHYSICS_HZ:.1f}s")
                return 0
            print("SMOKE TEST NEEDS TUNING: hunter failed to catch stationary player within test window")
            return 1
        return 0
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.exit(run())