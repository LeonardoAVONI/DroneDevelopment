#!/usr/bin/env python3
"""
mavsdk_reference.py
====================
A self-contained MAVSDK-Python reference script.
Run against PX4 SITL:  python mavsdk_reference.py

Prerequisites:
    pip install mavsdk

Start SITL first:
    make px4_sitl gazebo          # or gazebo-classic
    # PX4 exposes MAVLink on udp://:14540 by default
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityBodyYawspeed,
    VelocityNedYaw,
    AttitudeRate,
)
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import LandedState


# ─────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────

async def connect(address: str = "udp://:14540") -> System:
    """Create and connect a System instance."""
    drone = System()
    print(f"[connect] Connecting to {address} …")
    await drone.connect(system_address=address)

    # Block until the drone is actually discovered
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[connect] Connected!")
            break
    return drone


async def wait_for_global_position(drone: System) -> None:
    """Block until the estimator has a valid global position."""
    print("[health] Waiting for global position estimate …")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[health] Global position OK")
            break


async def print_health(drone: System) -> None:
    """Print one health snapshot."""
    async for health in drone.telemetry.health():
        print(
            f"[health] gps_ok={health.is_global_position_ok} "
            f"home_ok={health.is_home_position_ok} "
            f"accel_ok={health.is_accelerometer_calibration_ok} "
            f"gyro_ok={health.is_gyrometer_calibration_ok} "
            f"mag_ok={health.is_magnetometer_calibration_ok} "
            f"armable={health.is_armable}"
        )
        break  # just one sample


async def print_battery(drone: System) -> None:
    """Print one battery snapshot."""
    async for battery in drone.telemetry.battery():
        print(
            f"[battery] voltage={battery.voltage_v:.2f} V  "
            f"remaining={battery.remaining_percent * 100:.1f}%"
        )
        break


async def print_position(drone: System) -> None:
    """Print one GPS position snapshot."""
    async for pos in drone.telemetry.position():
        print(
            f"[position] lat={pos.latitude_deg:.7f}  "
            f"lon={pos.longitude_deg:.7f}  "
            f"alt_abs={pos.absolute_altitude_m:.2f} m  "
            f"alt_rel={pos.relative_altitude_m:.2f} m"
        )
        break


async def print_attitude(drone: System) -> None:
    """Print one attitude (Euler) snapshot."""
    async for att in drone.telemetry.attitude_euler():
        print(
            f"[attitude] roll={att.roll_deg:.2f}°  "
            f"pitch={att.pitch_deg:.2f}°  "
            f"yaw={att.yaw_deg:.2f}°"
        )
        break


async def print_velocity(drone: System) -> None:
    """Print one NED velocity snapshot."""
    async for vel in drone.telemetry.velocity_ned():
        print(
            f"[velocity] N={vel.north_m_s:.2f}  "
            f"E={vel.east_m_s:.2f}  "
            f"D={vel.down_m_s:.2f} m/s"
        )
        break


async def print_flight_mode(drone: System) -> None:
    """Print current flight mode."""
    async for mode in drone.telemetry.flight_mode():
        print(f"[mode] {mode}")
        break


async def all_checks(drone: System) -> None:
    """Run all telemetry checks in one go."""
    print("\n── Telemetry snapshot ──────────────────────────")
    await print_health(drone)
    await print_battery(drone)
    await print_position(drone)
    await print_attitude(drone)
    await print_velocity(drone)
    await print_flight_mode(drone)
    print("────────────────────────────────────────────────\n")


# ─────────────────────────────────────────────
#  ARM / DISARM
# ─────────────────────────────────────────────

async def arm(drone: System) -> None:
    print("[action] Arming …")
    try:
        await drone.action.arm()
        print("[action] Armed")
    except ActionError as e:
        print(f"[action] Arm failed: {e}")
        raise


async def disarm(drone: System) -> None:
    print("[action] Disarming …")
    try:
        await drone.action.disarm()
        print("[action] Disarmed")
    except ActionError as e:
        print(f"[action] Disarm failed: {e}")
        raise


# ─────────────────────────────────────────────
#  TAKEOFF / LAND / RTL
# ─────────────────────────────────────────────

async def takeoff(drone: System, altitude_m: float = 10.0) -> None:
    """Set takeoff altitude then command takeoff."""
    await drone.action.set_takeoff_altitude(altitude_m)
    print(f"[action] Taking off to {altitude_m} m …")
    await drone.action.takeoff()

    # Wait until we reach roughly the target altitude
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= altitude_m * 0.95:
            print(f"[action] Reached {pos.relative_altitude_m:.1f} m")
            break



async def land(drone: System) -> None:
    print("[action] Landing …")
    await drone.action.land()

    # Wait until physically on the ground
    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            print("[action] Touchdown detected")
            break

    # Now actually send the disarm
    await drone.action.disarm()
    print("[action] Disarmed")

async def return_to_launch(drone: System) -> None:
    """Command RTL and wait until disarmed."""
    print("[action] Return to launch …")
    await drone.action.return_to_launch()
    async for armed in drone.telemetry.armed():
        if not armed:
            print("[action] RTL complete, disarmed")
            break



async def goto_location(
    drone: System,
    lat: float,
    lon: float,
    alt_abs: float,
    yaw_deg: float = float("nan"),
) -> None:
    """Fly to an absolute GPS coordinate (AMSL altitude)."""
    print(f"[action] Going to ({lat:.6f}, {lon:.6f}, {alt_abs:.1f} m AMSL) …")
    await drone.action.goto_location(lat, lon, alt_abs, yaw_deg)


# ─────────────────────────────────────────────
#  OFFBOARD — position loop (NED frame)
# ─────────────────────────────────────────────

async def offboard_square_ned(drone: System, side_m: float = 20.0) -> None:
    """
    Fly a square in Offboard mode using NED position setpoints.
    The square is relative to the position when Offboard starts.
    """
    print("[offboard] Starting NED square …")

    # Must send at least one setpoint before enabling Offboard
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[offboard] Start failed: {e}")
        return

    corners = [
        (0.0,     0.0,     -10.0, 0.0),    # hover at start
        (side_m,  0.0,     -10.0, 0.0),    # North
        (side_m,  side_m,  -10.0, 90.0),   # NE
        (0.0,     side_m,  -10.0, 180.0),  # East back
        (0.0,     0.0,     -10.0, 270.0),  # back to start
    ]

    for n, e, d, yaw in corners:
        print(f"[offboard] NED setpoint: N={n} E={e} D={d} yaw={yaw}°")
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
        await asyncio.sleep(5)

    try:
        await drone.offboard.stop()
        print("[offboard] Offboard stopped")
    except OffboardError as e:
        print(f"[offboard] Stop failed: {e}")


# ─────────────────────────────────────────────
#  OFFBOARD — orbit using velocity setpoints
# ─────────────────────────────────────────────

async def offboard_orbit_velocity(
    drone: System,
    radius_m: float = 20.0,
    altitude_rel: float = 10.0,
    speed_mps: float = 3.0,
    turns: float = 1.0,
) -> None:
    """
    Fly a horizontal circle around the current position using
    NED velocity setpoints. The drone faces the direction of travel.
    """
    print(f"[offboard] Orbit: r={radius_m} m, alt={altitude_rel} m, v={speed_mps} m/s")

    # Circumference and total time
    circumference = 2 * math.pi * radius_m
    total_time = (circumference / speed_mps) * turns
    dt = 0.1  # setpoint update period [s]
    steps = int(total_time / dt)
    omega = speed_mps / radius_m  # angular velocity [rad/s]

    # Prime the setpoint before starting
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(speed_mps, 0.0, 0.0, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[offboard] Start failed: {e}")
        return

    for i in range(steps):
        t = i * dt
        angle = omega * t
        # Velocity tangent to the circle
        vn = speed_mps * math.cos(angle)
        ve = speed_mps * math.sin(angle)
        vd = 0.0
        yaw = math.degrees(angle) % 360

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, yaw)
        )
        await asyncio.sleep(dt)

    try:
        await drone.offboard.stop()
        print("[offboard] Orbit complete, Offboard stopped")
    except OffboardError as e:
        print(f"[offboard] Stop failed: {e}")


# ─────────────────────────────────────────────
#  MISSION API
# ─────────────────────────────────────────────

async def upload_and_run_mission(drone: System) -> None:
    """
    Upload a simple 3-waypoint mission and execute it.
    Waypoints are offset from a hard-coded home; adjust as needed.
    """

    # Get current position as reference
    async for pos in drone.telemetry.position():
        home_lat = pos.latitude_deg
        home_lon = pos.longitude_deg
        home_alt = pos.absolute_altitude_m
        break

    print(f"[mission] Building mission from home ({home_lat:.6f}, {home_lon:.6f})")

    def wp(lat_off: float, lon_off: float, rel_alt: float) -> MissionItem:
        """Helper to build a MissionItem with sensible defaults."""
        return MissionItem(
            latitude_deg=home_lat + lat_off,
            longitude_deg=home_lon + lon_off,
            relative_altitude_m=rel_alt,
            speed_m_s=5.0,
            is_fly_through=True,
            gimbal_pitch_deg=float("nan"),
            gimbal_yaw_deg=float("nan"),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float("nan"),
            camera_photo_interval_s=float("nan"),
            acceptance_radius_m=float("nan"),
            yaw_deg=float("nan"),
            camera_photo_distance_m=float("nan"),
        )

    mission_items = [
        wp(0.0005,  0.0,     15.0),   # WP1 — north
        wp(0.0005,  0.0005,  15.0),   # WP2 — NE
        wp(0.0,     0.0005,  15.0),   # WP3 — east
    ]

    mission_plan = MissionPlan(mission_items)

    print("[mission] Uploading …")
    await drone.mission.upload_mission(mission_plan)
    print("[mission] Upload complete")

    # Optional: enable RTL after mission
    await drone.mission.set_return_to_launch_after_mission(True)

    print("[mission] Starting mission …")
    await drone.mission.start_mission()

    # Monitor progress
    async for progress in drone.mission.mission_progress():
        print(
            f"[mission] Progress: {progress.current}/{progress.total}"
        )
        if progress.current == progress.total:
            print("[mission] Mission complete!")
            break


async def pause_and_resume_mission(drone: System) -> None:
    print("[mission] Pausing mission …")
    await drone.mission.pause_mission()
    await asyncio.sleep(3)
    print("[mission] Resuming mission …")
    await drone.mission.start_mission()


async def clear_mission(drone: System) -> None:
    print("[mission] Clearing mission …")
    await drone.mission.clear_mission()


# ─────────────────────────────────────────────
#  PARAM API  (read / write PX4 parameters)
# ─────────────────────────────────────────────

async def demo_params(drone: System) -> None:
    """Read and write a parameter as an example."""
    # Read
    mpc_xy_vel = await drone.param.get_param_float("MPC_XY_VEL_MAX")
    print(f"[param] MPC_XY_VEL_MAX = {mpc_xy_vel} m/s")

    # Write  (be careful in real flights!)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 5.0)
    print("[param] Set MPC_XY_VEL_MAX = 5.0 m/s")

    # Integer param example
    sys_id = await drone.param.get_param_int("MAV_SYS_ID")
    print(f"[param] MAV_SYS_ID = {sys_id}")


# ─────────────────────────────────────────────
#  TELEMETRY STREAMS (async generators)
# ─────────────────────────────────────────────

async def stream_telemetry_for(drone: System, seconds: float = 5.0) -> None:
    """Show how to read telemetry streams concurrently."""

    async def _pos():
        async for p in drone.telemetry.position():
            print(
                f"  [stream] lat={p.latitude_deg:.6f} "
                f"lon={p.longitude_deg:.6f} "
                f"alt_rel={p.relative_altitude_m:.1f} m"
            )

    async def _att():
        async for a in drone.telemetry.attitude_euler():
            print(
                f"  [stream] roll={a.roll_deg:.1f}° "
                f"pitch={a.pitch_deg:.1f}° "
                f"yaw={a.yaw_deg:.1f}°"
            )

    print(f"[stream] Streaming telemetry for {seconds} s …")
    pos_task = asyncio.ensure_future(_pos())
    att_task = asyncio.ensure_future(_att())
    await asyncio.sleep(seconds)
    pos_task.cancel()
    att_task.cancel()
    print("[stream] Done")


# ─────────────────────────────────────────────
#  MAIN FLIGHT DEMO
# ─────────────────────────────────────────────

async def main():
    # ── 1. Connect ───────────────────────────
    drone = await connect("udp://:14540")

    # ── 2. Pre-flight checks ─────────────────
    await wait_for_global_position(drone)
    await all_checks(drone)

    # ── 3. Param demo ─────────────────────────
    await demo_params(drone)

    # ── 4. Arm ───────────────────────────────
    await arm(drone)

    # ── 5. Takeoff ───────────────────────────
    await takeoff(drone, altitude_m=10.0)
    await asyncio.sleep(2)

    # ── 6. In-flight checks ──────────────────
    await all_checks(drone)

    # ── 7. Offboard orbit ────────────────────
    # Comment out whichever demo you don't want to run

    # Option A: NED square in Offboard
    # await offboard_square_ned(drone, side_m=15.0)

    # Option B: Circular orbit in Offboard
    await offboard_orbit_velocity(
        drone,
        radius_m=15.0,
        altitude_rel=10.0,
        speed_mps=3.0,
        turns=1.5,
    )

    # ── 8. Post-orbit checks ─────────────────
    await all_checks(drone)

    # ── 9. Upload & run a waypoint mission ───
    # await upload_and_run_mission(drone)

    # ── 10. Go to a specific GPS point ───────
    # async for pos in drone.telemetry.position(): break
    # await goto_location(drone, pos.latitude_deg + 0.001, pos.longitude_deg, pos.absolute_altitude_m)
    # await asyncio.sleep(8)

    # ── 11. Stream telemetry for 5 s ─────────
    await stream_telemetry_for(drone, seconds=5.0)

    # ── 12. Land ─────────────────────────────
    await land(drone)

    # Drone auto-disarms after landing; but you can force it:
    # await disarm(drone)

    # Alternative endings:
    # await return_to_launch(drone)


if __name__ == "__main__":
    asyncio.run(main())