/****************************************************************************
 * CUSTOM VOLIRO HEXA TILTING EFFECTIVENESS
 ****************************************************************************/

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

class ActuatorEffectivenessHexaTilting : public ModuleParams, public ActuatorEffectiveness
{
public:
    static constexpr int NUM_ROTORS_MAX = 12;

    ActuatorEffectivenessHexaTilting(ModuleParams *parent);
    virtual ~ActuatorEffectivenessHexaTilting() = default;

    bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
                        ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
                        const ActuatorVector &actuator_max) override;

    void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
    {
        allocation_method_out[0] = AllocationMethod::PSEUDO_INVERSE;
    }

    void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
    {
        normalize[0] = false;
    }

    const char *name() const override { return "HexaTilting"; }

private:
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    void updateParams() override;
    uint32_t _armed_cycles{0};

    struct ParamHandles {
        param_t position_x;
        param_t position_y;
        param_t moment_ratio;
    };

    ParamHandles _param_handles[NUM_ROTORS_MAX];

    struct RotorGeometry {
        float x;
        float y;
        float km;
    };

    float _angulo_acumulado[6]{0.0f};

    RotorGeometry _geometry[NUM_ROTORS_MAX];

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count
    )
};
