import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

async def run():
    # Inicializamos el dron y nos conectamos al simulador SITL
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Esperando a que el dron se conecte...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- ¡Dron detectado!")
            break

    print("-- Armando motores")
    await drone.action.arm()

    # PX4 requiere recibir comandos continuamente antes de entrar en Offboard
    print("-- Configurando setpoint inicial (requerido para Offboard)")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Iniciando modo Offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Error al iniciar Offboard: {error}")
        return

    # Eje Z invertido (negativo es hacia arriba)
    print("-- Ascendiendo (Z = -1.5 m/s) durante 4 segundos")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.5, 0.0))
    await asyncio.sleep(4)

    # LA MAGIA DEL OMNICÓPTERO
    # Ejes: (Adelante, Derecha, Abajo, Velocidad_Rotacion_Yaw)
    print("-- Moviéndose a la DERECHA a 2 m/s (sin usar Roll)")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 2.0, 0.0, 0.0))
    await asyncio.sleep(4)

    print("-- Moviéndose hacia ADELANTE a 2 m/s (sin usar Pitch)")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(2.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(4)

    print("-- Deteniendo y Aterrizando")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    
    await drone.offboard.stop()
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
