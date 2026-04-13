/****************************************************************************
 * CUSTOM VOLIRO HEXA TILTING EFFECTIVENESS
 ****************************************************************************/

#include "ActuatorEffectivenessHexaCopterTilting.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessHexaTilting::ActuatorEffectivenessHexaTilting(ModuleParams *parent)
    : ModuleParams(parent)
{
    // 1. Buscamos los parámetros CA_ROTORx_PX, PY y KM en el sistema
    for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
        _param_handles[i].position_x = param_find(buffer);

        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
        _param_handles[i].position_y = param_find(buffer);

        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
        _param_handles[i].moment_ratio = param_find(buffer);
    }

    // Inicializamos leyendo los valores actuales
    updateParams();
}

void ActuatorEffectivenessHexaTilting::updateParams()
{
    ModuleParams::updateParams();

    //int num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));
    // FUERZA UN LÍMITE SEGURO:
    int num_rotors = _param_ca_rotor_count.get();
    if (num_rotors < 0) num_rotors = 0;
    if (num_rotors > 6) num_rotors = 6; // Tu hexacóptero solo tiene 6 motores
    // 2. Extraemos los valores físicos a nuestra geometría local
    for (int i = 0; i < num_rotors; ++i) {
        param_get(_param_handles[i].position_x, &_geometry[i].x);
        param_get(_param_handles[i].position_y, &_geometry[i].y);
        param_get(_param_handles[i].moment_ratio, &_geometry[i].km);
    }
}

bool
ActuatorEffectivenessHexaTilting::getEffectivenessMatrix(Configuration &configuration,
        EffectivenessUpdateReason external_update)
{
    if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
        return false;
    }

    //configuration.reset();
    // 1. Le decimos a PX4 que nuestra matriz matemática resuelve 12 salidas
    //configuration.num_actuators_matrix[0] = 12;

    // Inicializa a cero la matriz (6x12 en tu caso)
    for (int i = 0; i < NUM_AXES; i++) {
        for (int j = 0; j < 12; j++) {
            configuration.effectiveness_matrices[0](i, j) = 0.0f;
        }
    }

    // 2. USAMOS LA API OFICIAL PARA REGISTRAR LOS ACTUADORES
    // Primero añadimos los 6 motores
    configuration.actuatorsAdded(ActuatorType::MOTORS, 6);

    // Y justo después, los 6 servos. (El orden es vital).
    configuration.actuatorsAdded(ActuatorType::SERVOS, 6);

    int num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));


    // 3. Rellenamos la matriz (Aquí va tu código actual)
    for (int i = 0; i < num_rotors; i++) {
        int v_idx = i;           // Fuerzas Verticales a los primeros 6 espacios (Motores)
        int l_idx = i + 6;       // Fuerzas Laterales a los 6 siguientes (Servos virtuales)


    //for (int i = 0; i < num_rotors; i++) {
     //   int v_idx = 2 * i;       // Índice columna Fuerza Vertical
     //   int l_idx = 2 * i + 1;   // Índice columna Fuerza Lateral

        // Obtenemos parámetros físicos de este rotor específico
        float x = _geometry[i].x;
        float y = _geometry[i].y;
        float km = _geometry[i].km; // NOTA: 'km' ya incluye el signo (CW/CCW)

        // Calculamos longitud del brazo y trigonometría dinámicamente
        // Evitamos división por cero si el rotor está en el centro
        float l = sqrtf(x * x + y * y);
        float c_phi = (l > 0.001f) ? (x / l) : 1.0f;
        float s_phi = (l > 0.001f) ? (y / l) : 0.0f;

        // Rellenamos la matriz usando la misma cinemática que tenías

        // FILA 0: Roll Moment (Mx)
        configuration.effectiveness_matrices[0](0, v_idx) = -l * s_phi;          // -y
        configuration.effectiveness_matrices[0](0, l_idx) = km * s_phi;

        // FILA 1: Pitch Moment (My)
        configuration.effectiveness_matrices[0](1, v_idx) = l * c_phi;           // x
        configuration.effectiveness_matrices[0](1, l_idx) = -km * c_phi;

        // FILA 2: Yaw Moment (Mz)
        configuration.effectiveness_matrices[0](2, v_idx) = km;
        configuration.effectiveness_matrices[0](2, l_idx) = l;

        // FILA 3: Fuerza X
        configuration.effectiveness_matrices[0](3, v_idx) = 0.0f;
        configuration.effectiveness_matrices[0](3, l_idx) = -s_phi;

        // FILA 4: Fuerza Y
        configuration.effectiveness_matrices[0](4, v_idx) = 0.0f;
        configuration.effectiveness_matrices[0](4, l_idx) = c_phi;

        // FILA 5: Fuerza Z (En NED, el empuje hacia arriba es negativo)
        configuration.effectiveness_matrices[0](5, v_idx) = -1.0f;
        configuration.effectiveness_matrices[0](5, l_idx) = 0.0f;
    }

    return true;
}

void ActuatorEffectivenessHexaTilting::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
                                                      int matrix_index,
                                                      ActuatorVector &actuator_sp,
                                                      const ActuatorVector &actuator_min,
                                                      const ActuatorVector &actuator_max)
{
    if (matrix_index != 0) {
        return;
    }

    float total_cmd_thrust = sqrtf(control_sp(3)*control_sp(3) +
                                   control_sp(4)*control_sp(4) +
                                   control_sp(5)*control_sp(5));

    for (int i = 0; i < 6; i++) {
        float F_vi = actuator_sp(i);
        float F_li = actuator_sp(i + 6);

        if (!PX4_ISFINITE(F_vi) || !PX4_ISFINITE(F_li)) {
            actuator_sp(i)     = 0.0f;
            actuator_sp(i + 6) = 0.0f;
            continue;
        }

        float motor_thrust = sqrtf(F_vi * F_vi + F_li * F_li);
        float desired;

        if (motor_thrust < 0.05f) {
            desired = _angulo_acumulado[i];

        } else if (total_cmd_thrust < 0.1f) {
            desired = 0.0f;
            if (F_vi < -0.02f) {
                motor_thrust = 0.0f;
            }

        } else {
            desired = atan2f(F_li, F_vi);
        }

        float error = desired - _angulo_acumulado[i];
        while (error >  M_PI_F) error -= 2.0f * M_PI_F;
        while (error < -M_PI_F) error += 2.0f * M_PI_F;
        _angulo_acumulado[i] += error;

        float servo_out = _angulo_acumulado[i] / M_PI_F;

        actuator_sp(i)     = math::constrain(motor_thrust, 0.0f, 1.0f);
        actuator_sp(i + 6) = PX4_ISFINITE(servo_out) ? math::constrain(servo_out, -1.0f, 1.0f) : 0.0f;

        PX4_INFO("Motor %d: F_vi=%.3f F_li=%.3f acum=%.3f out=%.3f",
            i, (double)F_vi, (double)F_li,
            (double)_angulo_acumulado[i],
            (double)actuator_sp(i + 6));
    }
}
