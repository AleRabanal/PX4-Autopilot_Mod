import time
from pymavlink import mavutil

def enviar_comando_offboard(master, q, thrust_z, thrust_y=0.0, thrust_x=0.0):
    # Máscara: 0b00000111 (Ignoramos las tasas de rotación, solo usamos el cuaternión de actitud)
    type_mask = 0b00000111
    
    try:
        # Intentamos usar la extensión de MAVLink para Empuje 3D (thrust_body)
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) % 4294967295, # time_boot_ms
            master.target_system,
            master.target_component,
            type_mask,
            q, # Cuaternión de actitud [w, x, y, z]
            0, 0, 0, # Tasas de Roll, Pitch, Yaw (ignoradas por la máscara)
            thrust_z, # Empuje vertical base (0 a 1)
            thrust_body=[thrust_x, thrust_y, -thrust_z] # Vector de empuje 3D [Adelante, Derecha, Abajo]
        )
    except TypeError:
        # Fallback: Si tu versión de pymavlink no tiene la extensión thrust_body compilada,
        # enviamos el comando estándar. El dron se mantendrá plano, pero no se moverá lateralmente.
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) % 4294967295,
            master.target_system, master.target_component,
            type_mask, q, 0, 0, 0, thrust_z
        )

def main():
    print("-- Conectando al SITL...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    master.wait_heartbeat()
    print("-- ¡Conectado!")

    # 1. Cambiar a modo Offboard y Armar
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 209, 6, 0, 0, 0, 0, 0)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    print("-- Motores armados. Entrando en Offboard.")

    # Cuaternión para actitud totalmente plana (0 inclinación)
    actitud_plana = [1.0, 0.0, 0.0, 0.0]
    
    # Empuje necesario para mantener el dron en el aire (Hover). En simulador suele ser ~0.3 a 0.5
    hover_thrust = 0.4 

    print("-- Despegando plano...")
    t_start = time.time()
    while time.time() - t_start < 4:
        enviar_comando_offboard(master, actitud_plana, hover_thrust)
        time.sleep(0.05) # Loop a 20Hz

    print("-- Moviéndose a la DERECHA (Empuje lateral puro)...")
    t_start = time.time()
    while time.time() - t_start < 4:
        # Añadimos un 20% de empuje (0.2) en el eje Y (Derecha) manteniendo la actitud plana
        enviar_comando_offboard(master, actitud_plana, hover_thrust, thrust_y=0.2)
        time.sleep(0.05)

    print("-- Deteniendo y Aterrizando...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

if __name__ == '__main__':
    main()
