import asyncio
import math
import random
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw, VelocityBodyYawspeed
from mavsdk.action import ActionError

async def wait_until_ready(drone, label):
    """Waits for the drone to connect and for all preflight checks to pass."""
    print(f"[{label}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] Connected to MAVLink!")
            break

    print(f"[{label}] Waiting for EKF/GPS convergence and preflight checks...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
            print(f"[{label}] Health checks passed! Ready to arm.")
            break
        await asyncio.sleep(1)

async def arm_with_retry(drone, label, retries=5):
    """Attempts to arm the drone, retrying if the commander temporarily denies it."""
    for attempt in range(1, retries + 1):
        try:
            await drone.action.arm()
            print(f"[{label}] Armed successfully!")
            return
        except ActionError as e:
            print(f"[{label}] Arming denied on attempt {attempt}: {e._result.result}")
            if attempt == retries:
                print(f"[{label}] Failed to arm after {retries} attempts.")
                raise e
            print(f"[{label}] Retrying in 2 seconds...")
            await asyncio.sleep(2)

async def get_position(drone):
    """Helper to fetch the latest position data."""
    async for pos in drone.telemetry.position():
        return pos

async def run_enemy(enemy, airborne_event):
    await wait_until_ready(enemy, "Enemy")

    print("[Enemy] Arming and taking off...")
    await arm_with_retry(enemy, "Enemy")
    await enemy.action.takeoff()
    await asyncio.sleep(8)  # Wait to reach safe altitude

    # Signal the Defender that the Enemy is in the air!
    print("[Enemy] Airborne. Signaling Defender...")
    airborne_event.set()

    # Start offboard with a neutral velocity
    await enemy.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await enemy.offboard.start()
        print("[Enemy] Offboard started. Executing randomized evasive maneuvers...")
    except OffboardError as e:
        print(f"[Enemy] Offboard failed: {e}")
        return

    # Random Evasive Maneuver Loop (runs for roughly 45 seconds)
    for _ in range(15):
        # Pick a random speed between 3 m/s and 10 m/s
        rand_speed = random.uniform(3.0, 10.0)
        # Pick a random turn rate between -30 (hard left) and +30 (hard right) degrees/sec
        rand_yaw = random.uniform(-30.0, 30.0)
        
        await enemy.offboard.set_velocity_body(VelocityBodyYawspeed(rand_speed, 0.0, 0.0, rand_yaw))
        
        # Hold this random vector for 3 seconds before juking again
        await asyncio.sleep(3)

    print("[Enemy] Mission complete. Attempting to land...")
    try:
        await enemy.offboard.stop()
        await enemy.action.land()
    except Exception as e:
        # If the Defender did its job, the Enemy is already dead.
        print(f"[Enemy] Cannot land. Aircraft destroyed: {e}")

async def run_defender(defender, enemy, airborne_event):
    await wait_until_ready(defender, "Defender")

    print("[Defender] Health checks passed. Holding on pad for target launch...")
    
    # Pause execution here until the Enemy triggers the event
    await airborne_event.wait()
    
    print("[Defender] Target airborne! Scrambling...")
    await arm_with_retry(defender, "Defender")
    await defender.action.takeoff()
    await asyncio.sleep(8)  # Wait to reach safe altitude

    # Start offboard with a 0 velocity setpoint
    await defender.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await defender.offboard.start()
        print("[Defender] Offboard started. Running intercept logic...")
    except OffboardError as e:
        print(f"[Defender] Offboard failed: {e._result.result}")
        return

    while True:
        enemy_pos = await get_position(enemy)
        defender_pos = await get_position(defender)

        # Calculate distances
        dlat = enemy_pos.latitude_deg - defender_pos.latitude_deg
        dlon = enemy_pos.longitude_deg - defender_pos.longitude_deg
        dalt = enemy_pos.absolute_altitude_m - defender_pos.absolute_altitude_m

        dn = dlat * 111000
        de = dlon * 111000 * math.cos(math.radians(defender_pos.latitude_deg))
        distance = math.sqrt(dn**2 + de**2)

        print(f"[Defender] Target Lock | Dist: {distance:.1f}m | Δalt: {dalt:.1f}m")

        # Kinetic Strike Radius
        if distance < 1:
            print("[Defender] *** KINETIC IMPACT !!! ***")
            break

        # Proportional velocity control (High Speed for impact)
        speed = 15.0
        scale = speed / max(distance, 0.1)
        vn = dn * scale
        ve = de * scale
        vd = -dalt * 0.5
        yaw = math.degrees(math.atan2(de, dn))

        await defender.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw))
        await asyncio.sleep(0.1)

    print("[Defender] Impact detected! Brace for Gazebo physics...")
    try:
        await defender.offboard.stop()
        await defender.action.land()
    except Exception as e:
        # Expected to fail due to tumbling physics after impact
        print(f"[Defender] Systems offline after impact (Expected behavior): {e}")

async def main():
    enemy = System(port=50051)
    defender = System(port=50052)

    await enemy.connect(system_address="udpin://127.0.0.1:14540")
    await defender.connect(system_address="udpin://127.0.0.1:14541")

    enemy_airborne_event = asyncio.Event()

    await asyncio.gather(
        run_enemy(enemy, enemy_airborne_event),
        run_defender(defender, enemy, enemy_airborne_event)
    )

if __name__ == "__main__":
    asyncio.run(main())