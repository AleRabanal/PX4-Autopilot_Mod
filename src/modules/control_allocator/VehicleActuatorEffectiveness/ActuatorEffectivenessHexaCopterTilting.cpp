/****************************************************************************
 * CUSTOM VOLIRO HEXA TILTING EFFECTIVENESS
 ****************************************************************************/

#include "ActuatorEffectivenessHexaCopterTilting.hpp"



#include <mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessHexaTilting::ActuatorEffectivenessHexaTilting(ModuleParams *parent)
    : ModuleParams(parent)
{


    for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
        _param_handles[i].position_x = param_find(buffer);

        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
        _param_handles[i].position_y = param_find(buffer);

        snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
        _param_handles[i].moment_ratio = param_find(buffer);
    }

    updateParams();
}

void ActuatorEffectivenessHexaTilting::updateParams()
{
    ModuleParams::updateParams();

    int num_rotors = _param_ca_rotor_count.get();
    if (num_rotors < 0) num_rotors = 0;
    if (num_rotors > 6) num_rotors = 6;

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

    // Inicializa a cero la matriz
    for (int i = 0; i < NUM_AXES; i++) {
        for (int j = 0; j < 12; j++) {
            configuration.effectiveness_matrices[0](i, j) = 0.0f;
        }
    }

    // Registramos 6 motores + 6 servos
    configuration.actuatorsAdded(ActuatorType::MOTORS, 6);
    configuration.actuatorsAdded(ActuatorType::SERVOS, 6);

    int num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));

    for (int i = 0; i < num_rotors; i++) {
        int v_idx = i;       // columna motor  (empuje vertical/axial)
        int l_idx = i + 6;   // columna servo  (empuje lateral al inclinar)

        float x  = _geometry[i].x;
        float y  = _geometry[i].y;
        float km = _geometry[i].km; // incluye signo CW/CCW

        float l     = sqrtf(x * x + y * y);
        float c_phi = (l > 0.001f) ? (x / l) : 1.0f;
        float s_phi = (l > 0.001f) ? (y / l) : 0.0f;

        // FILA 0: Roll (Mx)  — fuerza vertical a distancia y
        configuration.effectiveness_matrices[0](0, v_idx) = -y;
        configuration.effectiveness_matrices[0](0, l_idx) =  km * s_phi;

        // FILA 1: Pitch (My) — fuerza vertical a distancia x
        configuration.effectiveness_matrices[0](1, v_idx) =  x;
        configuration.effectiveness_matrices[0](1, l_idx) = -km * c_phi;

        // FILA 2: Yaw (Mz)
        // - Motor vertical: solo reacción de torque km (signo CW/CCW)
        // - Servo inclinado: fuerza lateral genera momento en Z

        configuration.effectiveness_matrices[0](2, v_idx) =  km;
        configuration.effectiveness_matrices[0](2, l_idx) =  x * c_phi + y * s_phi;

        // FILA 3: Fuerza X  — solo el servo inclinado contribuye
        configuration.effectiveness_matrices[0](3, v_idx) =  0.0f;
        configuration.effectiveness_matrices[0](3, l_idx) = -s_phi;

        // FILA 4: Fuerza Y
        configuration.effectiveness_matrices[0](4, v_idx) =  0.0f;
        configuration.effectiveness_matrices[0](4, l_idx) =  c_phi;

        // FILA 5: Fuerza Z (NED: empuje hacia arriba es negativo)
        configuration.effectiveness_matrices[0](5, v_idx) = -1.0f;
        configuration.effectiveness_matrices[0](5, l_idx) =  0.0f;
    }

    return true;
}

#include <px4_platform_common/log.h>

void ActuatorEffectivenessHexaTilting::updateSetpoint(
    const matrix::Vector<float, NUM_AXES> &control_sp,
    int matrix_index,
    ActuatorVector &actuator_sp,
    const ActuatorVector &actuator_min,
    const ActuatorVector &actuator_max)
{
    if (matrix_index != 0) return;

    vehicle_status_s status;
    _vehicle_status_sub.copy(&status);
    bool armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

    static bool modo_vuelo = false;

    if (!armed) {
        modo_vuelo = false;

        for (int i = 0; i < 12; i++) {
            actuator_sp(i) = 0.0f;
            if (i < 6) _angulo_acumulado[i] = 0.0f;
        }
        _armed_cycles = 0;
        return;
    }

    _armed_cycles++;

    float throttle = -control_sp(5);

    if (_armed_cycles % 100 == 0) {
        PX4_INFO("DEBUG - Ciclos: %u | ModoVuelo: %d | Throttle: %.3f",
                 (unsigned int)_armed_cycles, modo_vuelo, (double)throttle);
    }

    // ==========================================
    // 3. TRANSICIÓN DE ESTADOS (La llave de paso)
    // ==========================================
    if (!modo_vuelo && throttle > 0.15f) {
        modo_vuelo = true;
        PX4_INFO(">>> TRANSICION A MODO VUELO ACTIVADA <<<");
    }
    // NUEVO: Apagar el modo vuelo al aterrizar (cuando el throttle baja al ralentí)
    else if (modo_vuelo && throttle < 0.08f) {
        modo_vuelo = false;
        PX4_INFO(">>> TRANSICION A MODO SUELO (ATERRIZAJE) <<<");
    }

    // ==========================================
    // 4. LÓGICA DE CONTROL
    // ==========================================
    const float max_angle = 3.0f * 2.0f * M_PI_F;
    const float max_step  = 0.18f;

    for (int i = 0; i < 6; i++) {
        float motor_thrust;
        float desired_rad;

        if (!modo_vuelo) {
            // MODO SUELO: Bloqueado en ralentí.
            motor_thrust = 0.05f;
            desired_rad  = 0.0f; // Fuerza a los motores a mirar hacia arriba

            actuator_sp(i) = 0.0f;
            actuator_sp(i + 6) = 0.0f;
        } else {
            // MODO VUELO: Leemos los actuadores
            float F_vi = actuator_sp(i);
            float F_li = actuator_sp(i + 6);

            if (!PX4_ISFINITE(F_vi) || !PX4_ISFINITE(F_li)) {
                motor_thrust = 0.05f;
                desired_rad = _angulo_acumulado[i];
            } else {
                motor_thrust = sqrtf(F_vi * F_vi + F_li * F_li);

                if (motor_thrust < 0.05f) motor_thrust = 0.05f;

                // AJUSTE: Subimos este umbral de 0.01 a 0.05.
                // Evita que "ruido" pequeño en el suelo haga que los motores bailen.
                if (motor_thrust > 0.05f) {
                    desired_rad = atan2f(F_li, F_vi);
                } else {
                    desired_rad = _angulo_acumulado[i];
                }
            }
        }

        // ==========================================
        // 5. MOVIMIENTO SUAVE DE SERVOS Y SALIDA
        // ==========================================
        float error = desired_rad - _angulo_acumulado[i];
        while (error >  M_PI_F) error -= 2.0f * M_PI_F;
        while (error < -M_PI_F) error += 2.0f * M_PI_F;

        error = math::constrain(error, -max_step, max_step);
        _angulo_acumulado[i] += error;

        actuator_sp(i)     = math::constrain(motor_thrust, 0.0f, 1.0f);
        actuator_sp(i + 6) = math::constrain(_angulo_acumulado[i] / max_angle, -1.0f, 1.0f);
    }
}
