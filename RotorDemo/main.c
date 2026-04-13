#include <stdio.h>
#include <string.h>

#include "../include/Rotor.h"

static RotorCreateInput BuildMainRotorCreateInput(void)
{
    RotorCreateInput create_input;
    RotorConfig* rotor;
    RotorGearConfig* gear;

    Rotor_InitCreateInput(&create_input);
    rotor = &create_input.rotor;
    gear = &create_input.gear;

    strncpy(rotor->name, "main", sizeof(rotor->name) - 1);
    rotor->name[sizeof(rotor->name) - 1] = '\0';
    rotor->base[0] = 0.0f;
    rotor->base[1] = 0.0f;
    rotor->base[2] = 1.5f;
    rotor->normal[0] = 0.0f;
    rotor->normal[1] = 0.0f;
    rotor->normal[2] = 1.0f;
    rotor->forward[0] = 1.0f;
    rotor->forward[1] = 0.0f;
    rotor->forward[2] = 0.0f;
    rotor->diameter_m = 10.2f;
    rotor->weight_per_blade_lb = 44.0f;
    rotor->num_blades = 4.0f;
    rotor->rel_blade_center = 0.7f;
    rotor->dynamic = 0.7f;
    rotor->delta3 = 0.0f;
    rotor->delta = 1.0f;
    rotor->rpm = 424.0f;
    rotor->max_collective_deg = 15.8f;
    rotor->min_collective_deg = -0.2f;
    rotor->max_cyclic_ail_deg = 7.6f;
    rotor->max_cyclic_ele_deg = 4.94f;
    rotor->min_cyclic_ail_deg = -7.6f;
    rotor->min_cyclic_ele_deg = -4.94f;
    rotor->pitch_a_deg = 10.0f;
    rotor->pitch_b_deg = 10.0f;
    rotor->force_at_pitch_a_lb = 3000.0f;
    rotor->power_at_pitch_0_kw = 300.0f;
    rotor->power_at_pitch_b_kw = 3000.0f;
    rotor->airfoil_lift_coefficient = 6.0f;
    rotor->airfoil_drag_coefficient0 = 0.012f;
    rotor->airfoil_drag_coefficient1 = 0.08f;
    rotor->number_of_parts = 16.0f;
    rotor->rotor_correction_factor = 0.65f;

    gear->max_power_engine_kw = 650.0f;
    gear->engine_prop_factor = 0.05f;
    gear->max_power_rotor_brake_kw = 20.0f;
    gear->rotorgear_friction_kw = 1.0f;
    gear->engine_accel_limit = 0.05f;

    return create_input;
}

static RotorControlInput BuildHoverControl(void)
{
    RotorControlInput control;
    Rotor_InitControlInput(&control);
    control.collective = 0.35f;
    control.cyclic_ail = 0.0f;
    control.cyclic_ele = 0.0f;
    control.tilt_roll = 0.0f;
    control.tilt_pitch = 0.0f;
    control.tilt_yaw = 0.0f;
    control.balance = 1.0f;
    return control;
}

static RotorStepInput BuildBaseStepInput(void)
{
    RotorStepInput input;
    Rotor_InitStepInput(&input);
    input.state.pos[0] = 0.0;
    input.state.pos[1] = 0.0;
    input.state.pos[2] = 6378137.0;
    input.rho = 1.184f;
    input.engine_on = 1;
    input.rel_target = 1.0f;
    input.max_rel_torque = 1.0f;
    return input;
}

static void PrintVec3(const char* label, const float* v)
{
    printf("%s: [%.3f, %.3f, %.3f]\n", label, v[0], v[1], v[2]);
}

static void RunScenario(RotorContext* context, int idx, const char* name,
    const RotorStepInput* input)
{
    RotorOutput output;
    Rotor_InitOutput(&output);
    printf("Scenario: %s\n", name);
    if(!Rotor_Step(context, idx, input)) {
        printf("  step failed\n");
        return;
    }
    if(!Rotor_GetOutput(context, idx, &output)) {
        printf("  step failed\n");
        return;
    }

    PrintVec3("  total_force_local", output.total_force);
    PrintVec3("  total_moment_local", output.total_moment);
    PrintVec3("  gear_torque_local", output.gear_torque);
    PrintVec3("  rotor_force_local", output.rotor.force);
    PrintVec3("  rotor_moment_local", output.rotor.moment);
    printf("  rotor_scalar_torque: %.3f\n", output.rotor.scalar_torque);
    printf("  rotor_rpm: %.3f\n", output.rotor.rpm);
    printf("  rotor_omega_rel(current/next): %.3f / %.3f\n",
        output.rotor.omega_rel, output.rotor.omega_rel_next);
    printf("\n");
}

int main(void)
{
    RotorContext* context;
    RotorCreateInput create_input;
    int main_rotor_idx;
    RotorStepInput hover;
    RotorControlInput hover_control;
    RotorStepInput forward_flight;
    RotorControlInput forward_control;
    int i;

    context = Rotor_CreateContext();
    create_input = BuildMainRotorCreateInput();
    main_rotor_idx = Rotor_CreateRotor(context, &create_input);
    if(main_rotor_idx < 0) {
        fprintf(stderr, "Failed to create main rotor model.\n");
        Rotor_DestroyContext(context);
        return 1;
    }

    hover = BuildBaseStepInput();
    hover_control = BuildHoverControl();
    Rotor_SetControl(context, main_rotor_idx, &hover_control);

    for(i = 0; i < 120; ++i) {
        Rotor_Step(context, main_rotor_idx, &hover);
    }

    RunScenario(context, main_rotor_idx, "hover", &hover);

    forward_flight = hover;
    forward_flight.state.v[0] = 25.0f;
    forward_control = BuildHoverControl();
    forward_control.cyclic_ele = 0.05f;
    Rotor_SetControl(context, main_rotor_idx, &forward_control);
    RunScenario(context, main_rotor_idx, "forward_flight", &forward_flight);

    Rotor_DestroyContext(context);
    return 0;
}
