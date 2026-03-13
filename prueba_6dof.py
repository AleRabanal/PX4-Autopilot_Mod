import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    await drone.action.arm()

    # setpoint inicial obligatorio
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
    )

    await drone.offboard.start()

    print("Ascending")

    for _ in range(100):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
        )
        await asyncio.sleep(0.05)

    print("Moving sideways without tilt")

    while True:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

asyncio.run(run())

