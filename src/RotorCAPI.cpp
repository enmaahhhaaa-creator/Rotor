#include "../include/Rotor.h"

#include <float.h>
#include <cstring>
#include <string>

#include "RotorManagerInternal.hpp"

namespace {

struct RotorContextImpl {
    rotor::RotorManager manager;
};

void Zero3(float* v)
{
    v[0] = v[1] = v[2] = 0.0f;
}

void Zero3d(double* v)
{
    v[0] = v[1] = v[2] = 0.0;
}

void Copy3(const float* src, float* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

void Copy3d(const double* src, double* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

void Copy9(const float* src, float* dst)
{
    int i;
    for(i = 0; i < 9; ++i) {
        dst[i] = src[i];
    }
}

void SetCString(const std::string& src, char* dst, std::size_t dst_size)
{
    if(dst_size == 0) {
        return;
    }
    std::size_t copy_len = src.size();
    if(copy_len >= dst_size) {
        copy_len = dst_size - 1;
    }
    std::memcpy(dst, src.c_str(), copy_len);
    dst[copy_len] = '\0';
}

std::string GetCString(const char* src)
{
    if(src == 0) {
        return std::string();
    }
    return std::string(src);
}

bool IsFiniteFloat(float value)
{
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}

bool IsFiniteDouble(double value)
{
    return value == value && value >= -DBL_MAX && value <= DBL_MAX;
}

bool HasFiniteFloatArray(const float* values, int count)
{
    int i;
    for(i = 0; i < count; ++i) {
        if(!IsFiniteFloat(values[i])) {
            return false;
        }
    }
    return true;
}

bool HasFiniteDoubleArray(const double* values, int count)
{
    int i;
    for(i = 0; i < count; ++i) {
        if(!IsFiniteDouble(values[i])) {
            return false;
        }
    }
    return true;
}

float AbsFloat(float value)
{
    return value < 0.0f ? -value : value;
}

bool HasUsableDirection(const float* value)
{
    double length2;
    if(!HasFiniteFloatArray(value, 3)) {
        return false;
    }

    length2 = (double)value[0] * (double)value[0]
        + (double)value[1] * (double)value[1]
        + (double)value[2] * (double)value[2];
    return length2 > 1e-12;
}

bool ValidateRotorConfig(const RotorConfig& config)
{
#define CHECK_FINITE_FLOAT(value) if(!IsFiniteFloat(value)) return false
    if(!HasFiniteFloatArray(config.base, 3)
        || !HasUsableDirection(config.normal)
        || !HasUsableDirection(config.forward)
        || !HasFiniteFloatArray(config.tilt_center, 3)) {
        return false;
    }

    CHECK_FINITE_FLOAT(config.max_cyclic_ail_deg);
    CHECK_FINITE_FLOAT(config.max_cyclic_ele_deg);
    CHECK_FINITE_FLOAT(config.min_cyclic_ail_deg);
    CHECK_FINITE_FLOAT(config.min_cyclic_ele_deg);
    CHECK_FINITE_FLOAT(config.max_collective_deg);
    CHECK_FINITE_FLOAT(config.min_collective_deg);
    CHECK_FINITE_FLOAT(config.diameter_m);
    CHECK_FINITE_FLOAT(config.weight_per_blade_lb);
    CHECK_FINITE_FLOAT(config.num_blades);
    CHECK_FINITE_FLOAT(config.rel_blade_center);
    CHECK_FINITE_FLOAT(config.dynamic);
    CHECK_FINITE_FLOAT(config.delta3);
    CHECK_FINITE_FLOAT(config.delta);
    CHECK_FINITE_FLOAT(config.translift);
    CHECK_FINITE_FLOAT(config.drag_factor);
    CHECK_FINITE_FLOAT(config.steps_per_second);
    CHECK_FINITE_FLOAT(config.phi0_rad);
    CHECK_FINITE_FLOAT(config.rpm);
    CHECK_FINITE_FLOAT(config.rel_len_flap_hinge);
    CHECK_FINITE_FLOAT(config.flap0_rad);
    CHECK_FINITE_FLOAT(config.flapmin_rad);
    CHECK_FINITE_FLOAT(config.flapmax_rad);
    CHECK_FINITE_FLOAT(config.flap0_factor);
    CHECK_FINITE_FLOAT(config.teeter_damp);
    CHECK_FINITE_FLOAT(config.max_teeter_damp);
    CHECK_FINITE_FLOAT(config.rel_len_teeter_hinge);
    CHECK_FINITE_FLOAT(config.balance);
    CHECK_FINITE_FLOAT(config.min_tilt_yaw_deg);
    CHECK_FINITE_FLOAT(config.min_tilt_pitch_deg);
    CHECK_FINITE_FLOAT(config.min_tilt_roll_deg);
    CHECK_FINITE_FLOAT(config.max_tilt_yaw_deg);
    CHECK_FINITE_FLOAT(config.max_tilt_pitch_deg);
    CHECK_FINITE_FLOAT(config.max_tilt_roll_deg);
    CHECK_FINITE_FLOAT(config.downwash_factor);
    CHECK_FINITE_FLOAT(config.pitch_a_deg);
    CHECK_FINITE_FLOAT(config.pitch_b_deg);
    CHECK_FINITE_FLOAT(config.force_at_pitch_a_lb);
    CHECK_FINITE_FLOAT(config.power_at_pitch_0_kw);
    CHECK_FINITE_FLOAT(config.power_at_pitch_b_kw);
    CHECK_FINITE_FLOAT(config.translift_ve);
    CHECK_FINITE_FLOAT(config.translift_maxfactor);
    CHECK_FINITE_FLOAT(config.ground_effect_constant);
    CHECK_FINITE_FLOAT(config.vortex_state_lift_factor);
    CHECK_FINITE_FLOAT(config.vortex_state_c1);
    CHECK_FINITE_FLOAT(config.vortex_state_c2);
    CHECK_FINITE_FLOAT(config.vortex_state_c3);
    CHECK_FINITE_FLOAT(config.vortex_state_e1);
    CHECK_FINITE_FLOAT(config.vortex_state_e2);
    CHECK_FINITE_FLOAT(config.twist_deg);
    CHECK_FINITE_FLOAT(config.number_of_segments);
    CHECK_FINITE_FLOAT(config.number_of_parts);
    CHECK_FINITE_FLOAT(config.rel_len_where_incidence_is_measured);
    CHECK_FINITE_FLOAT(config.chord);
    CHECK_FINITE_FLOAT(config.taper);
    CHECK_FINITE_FLOAT(config.airfoil_incidence_no_lift_deg);
    CHECK_FINITE_FLOAT(config.rel_len_blade_start);
    CHECK_FINITE_FLOAT(config.incidence_stall_zero_speed_deg);
    CHECK_FINITE_FLOAT(config.incidence_stall_half_sonic_speed_deg);
    CHECK_FINITE_FLOAT(config.lift_factor_stall);
    CHECK_FINITE_FLOAT(config.stall_change_over_deg);
    CHECK_FINITE_FLOAT(config.drag_factor_stall);
    CHECK_FINITE_FLOAT(config.airfoil_lift_coefficient);
    CHECK_FINITE_FLOAT(config.airfoil_drag_coefficient0);
    CHECK_FINITE_FLOAT(config.airfoil_drag_coefficient1);
    CHECK_FINITE_FLOAT(config.cyclic_factor);
    CHECK_FINITE_FLOAT(config.rotor_correction_factor);

    if(config.diameter_m <= 0.0f
        || config.weight_per_blade_lb <= 0.0f
        || config.num_blades <= 0.0f
        || config.rel_blade_center <= 0.0f
        || config.steps_per_second <= 0.0f
        || config.rpm <= 0.0f
        || AbsFloat(config.pitch_a_deg) <= 1e-6f
        || AbsFloat(config.delta) <= 1e-6f
        || config.number_of_segments <= 0.0f
        || config.number_of_parts <= 0.0f
        || config.chord <= 0.0f
        || config.rel_len_flap_hinge <= -1.0f
        || config.rel_len_flap_hinge >= 1.0f) {
        return false;
    }

    if(!config.no_torque && AbsFloat(config.pitch_b_deg) <= 1e-6f) {
        return false;
    }

    return true;
#undef CHECK_FINITE_FLOAT
}

bool ValidateGearConfig(const RotorGearConfig& config)
{
    if(!IsFiniteFloat(config.max_power_engine_kw)
        || !IsFiniteFloat(config.engine_prop_factor)
        || !IsFiniteFloat(config.yasim_drag_factor)
        || !IsFiniteFloat(config.yasim_lift_factor)
        || !IsFiniteFloat(config.max_power_rotor_brake_kw)
        || !IsFiniteFloat(config.rotorgear_friction_kw)
        || !IsFiniteFloat(config.engine_accel_limit)
        || !IsFiniteFloat(config.initial_rel_rpm)) {
        return false;
    }

    if(config.max_power_engine_kw < 0.0f
        || AbsFloat(config.engine_prop_factor) <= 1e-6f
        || config.max_power_rotor_brake_kw < 0.0f
        || config.rotorgear_friction_kw < 0.0f
        || config.engine_accel_limit <= 0.0f
        || config.initial_rel_rpm < 0.0f) {
        return false;
    }

    return true;
}

bool ValidateControlInput(const RotorControlInput& control)
{
    return IsFiniteFloat(control.collective)
        && IsFiniteFloat(control.cyclic_ail)
        && IsFiniteFloat(control.cyclic_ele)
        && IsFiniteFloat(control.tilt_roll)
        && IsFiniteFloat(control.tilt_pitch)
        && IsFiniteFloat(control.tilt_yaw)
        && IsFiniteFloat(control.balance);
}

bool ValidateStepInput(const RotorStepInput& input)
{
    if(!HasFiniteDoubleArray(input.state.pos, 3)
        || !HasFiniteFloatArray(input.state.orient, 9)
        || !HasFiniteFloatArray(input.state.v, 3)
        || !HasFiniteFloatArray(input.state.rot, 3)
        || !HasFiniteFloatArray(input.wind_global, 3)
        || !HasFiniteFloatArray(input.cg_local, 3)
        || !IsFiniteFloat(input.dt)
        || !IsFiniteFloat(input.rho)
        || !IsFiniteFloat(input.rotor_brake)
        || !IsFiniteFloat(input.max_rel_torque)
        || !IsFiniteFloat(input.rel_target)) {
        return false;
    }

    if(input.dt <= 0.0f || input.rho < 0.0f) {
        return false;
    }

    return true;
}

void ToCpp(const RotorControlInput& c_in, rotor::RotorControlInput& cpp_out)
{
    cpp_out.collective = c_in.collective;
    cpp_out.cyclic_ail = c_in.cyclic_ail;
    cpp_out.cyclic_ele = c_in.cyclic_ele;
    cpp_out.tilt_roll = c_in.tilt_roll;
    cpp_out.tilt_pitch = c_in.tilt_pitch;
    cpp_out.tilt_yaw = c_in.tilt_yaw;
    cpp_out.balance = c_in.balance;
}

void ToC(const rotor::RotorControlInput& cpp_in, RotorControlInput& c_out)
{
    c_out.collective = cpp_in.collective;
    c_out.cyclic_ail = cpp_in.cyclic_ail;
    c_out.cyclic_ele = cpp_in.cyclic_ele;
    c_out.tilt_roll = cpp_in.tilt_roll;
    c_out.tilt_pitch = cpp_in.tilt_pitch;
    c_out.tilt_yaw = cpp_in.tilt_yaw;
    c_out.balance = cpp_in.balance;
}

void ToCpp(const RotorConfig& c_in, rotor::RotorConfig& cpp_out)
{
    Copy3(c_in.base, cpp_out.base);
    Copy3(c_in.normal, cpp_out.normal);
    Copy3(c_in.forward, cpp_out.forward);
    cpp_out.max_cyclic_ail_deg = c_in.max_cyclic_ail_deg;
    cpp_out.max_cyclic_ele_deg = c_in.max_cyclic_ele_deg;
    cpp_out.min_cyclic_ail_deg = c_in.min_cyclic_ail_deg;
    cpp_out.min_cyclic_ele_deg = c_in.min_cyclic_ele_deg;
    cpp_out.max_collective_deg = c_in.max_collective_deg;
    cpp_out.min_collective_deg = c_in.min_collective_deg;
    cpp_out.diameter_m = c_in.diameter_m;
    cpp_out.weight_per_blade_lb = c_in.weight_per_blade_lb;
    cpp_out.num_blades = c_in.num_blades;
    cpp_out.rel_blade_center = c_in.rel_blade_center;
    cpp_out.dynamic = c_in.dynamic;
    cpp_out.delta3 = c_in.delta3;
    cpp_out.delta = c_in.delta;
    cpp_out.translift = c_in.translift;
    cpp_out.drag_factor = c_in.drag_factor;
    cpp_out.steps_per_second = c_in.steps_per_second;
    cpp_out.phi0_rad = c_in.phi0_rad;
    cpp_out.rpm = c_in.rpm;
    cpp_out.rel_len_flap_hinge = c_in.rel_len_flap_hinge;
    cpp_out.flap0_rad = c_in.flap0_rad;
    cpp_out.flapmin_rad = c_in.flapmin_rad;
    cpp_out.flapmax_rad = c_in.flapmax_rad;
    cpp_out.flap0_factor = c_in.flap0_factor;
    cpp_out.teeter_damp = c_in.teeter_damp;
    cpp_out.max_teeter_damp = c_in.max_teeter_damp;
    cpp_out.rel_len_teeter_hinge = c_in.rel_len_teeter_hinge;
    cpp_out.balance = c_in.balance;
    cpp_out.min_tilt_yaw_deg = c_in.min_tilt_yaw_deg;
    cpp_out.min_tilt_pitch_deg = c_in.min_tilt_pitch_deg;
    cpp_out.min_tilt_roll_deg = c_in.min_tilt_roll_deg;
    cpp_out.max_tilt_yaw_deg = c_in.max_tilt_yaw_deg;
    cpp_out.max_tilt_pitch_deg = c_in.max_tilt_pitch_deg;
    cpp_out.max_tilt_roll_deg = c_in.max_tilt_roll_deg;
    Copy3(c_in.tilt_center, cpp_out.tilt_center);
    cpp_out.downwash_factor = c_in.downwash_factor;
    cpp_out.ccw = c_in.ccw != 0;
    cpp_out.shared_flap_hinge = c_in.shared_flap_hinge != 0;
    cpp_out.no_torque = c_in.no_torque != 0;
    cpp_out.name = GetCString(c_in.name);
    cpp_out.pitch_a_deg = c_in.pitch_a_deg;
    cpp_out.pitch_b_deg = c_in.pitch_b_deg;
    cpp_out.force_at_pitch_a_lb = c_in.force_at_pitch_a_lb;
    cpp_out.power_at_pitch_0_kw = c_in.power_at_pitch_0_kw;
    cpp_out.power_at_pitch_b_kw = c_in.power_at_pitch_b_kw;
    cpp_out.translift_ve = c_in.translift_ve;
    cpp_out.translift_maxfactor = c_in.translift_maxfactor;
    cpp_out.ground_effect_constant = c_in.ground_effect_constant;
    cpp_out.vortex_state_lift_factor = c_in.vortex_state_lift_factor;
    cpp_out.vortex_state_c1 = c_in.vortex_state_c1;
    cpp_out.vortex_state_c2 = c_in.vortex_state_c2;
    cpp_out.vortex_state_c3 = c_in.vortex_state_c3;
    cpp_out.vortex_state_e1 = c_in.vortex_state_e1;
    cpp_out.vortex_state_e2 = c_in.vortex_state_e2;
    cpp_out.twist_deg = c_in.twist_deg;
    cpp_out.number_of_segments = c_in.number_of_segments;
    cpp_out.number_of_parts = c_in.number_of_parts;
    cpp_out.rel_len_where_incidence_is_measured = c_in.rel_len_where_incidence_is_measured;
    cpp_out.chord = c_in.chord;
    cpp_out.taper = c_in.taper;
    cpp_out.airfoil_incidence_no_lift_deg = c_in.airfoil_incidence_no_lift_deg;
    cpp_out.rel_len_blade_start = c_in.rel_len_blade_start;
    cpp_out.incidence_stall_zero_speed_deg = c_in.incidence_stall_zero_speed_deg;
    cpp_out.incidence_stall_half_sonic_speed_deg = c_in.incidence_stall_half_sonic_speed_deg;
    cpp_out.lift_factor_stall = c_in.lift_factor_stall;
    cpp_out.stall_change_over_deg = c_in.stall_change_over_deg;
    cpp_out.drag_factor_stall = c_in.drag_factor_stall;
    cpp_out.airfoil_lift_coefficient = c_in.airfoil_lift_coefficient;
    cpp_out.airfoil_drag_coefficient0 = c_in.airfoil_drag_coefficient0;
    cpp_out.airfoil_drag_coefficient1 = c_in.airfoil_drag_coefficient1;
    cpp_out.cyclic_factor = c_in.cyclic_factor;
    cpp_out.rotor_correction_factor = c_in.rotor_correction_factor;
}

void ToC(const rotor::RotorConfig& cpp_in, RotorConfig& c_out)
{
    Copy3(cpp_in.base, c_out.base);
    Copy3(cpp_in.normal, c_out.normal);
    Copy3(cpp_in.forward, c_out.forward);
    c_out.max_cyclic_ail_deg = cpp_in.max_cyclic_ail_deg;
    c_out.max_cyclic_ele_deg = cpp_in.max_cyclic_ele_deg;
    c_out.min_cyclic_ail_deg = cpp_in.min_cyclic_ail_deg;
    c_out.min_cyclic_ele_deg = cpp_in.min_cyclic_ele_deg;
    c_out.max_collective_deg = cpp_in.max_collective_deg;
    c_out.min_collective_deg = cpp_in.min_collective_deg;
    c_out.diameter_m = cpp_in.diameter_m;
    c_out.weight_per_blade_lb = cpp_in.weight_per_blade_lb;
    c_out.num_blades = cpp_in.num_blades;
    c_out.rel_blade_center = cpp_in.rel_blade_center;
    c_out.dynamic = cpp_in.dynamic;
    c_out.delta3 = cpp_in.delta3;
    c_out.delta = cpp_in.delta;
    c_out.translift = cpp_in.translift;
    c_out.drag_factor = cpp_in.drag_factor;
    c_out.steps_per_second = cpp_in.steps_per_second;
    c_out.phi0_rad = cpp_in.phi0_rad;
    c_out.rpm = cpp_in.rpm;
    c_out.rel_len_flap_hinge = cpp_in.rel_len_flap_hinge;
    c_out.flap0_rad = cpp_in.flap0_rad;
    c_out.flapmin_rad = cpp_in.flapmin_rad;
    c_out.flapmax_rad = cpp_in.flapmax_rad;
    c_out.flap0_factor = cpp_in.flap0_factor;
    c_out.teeter_damp = cpp_in.teeter_damp;
    c_out.max_teeter_damp = cpp_in.max_teeter_damp;
    c_out.rel_len_teeter_hinge = cpp_in.rel_len_teeter_hinge;
    c_out.balance = cpp_in.balance;
    c_out.min_tilt_yaw_deg = cpp_in.min_tilt_yaw_deg;
    c_out.min_tilt_pitch_deg = cpp_in.min_tilt_pitch_deg;
    c_out.min_tilt_roll_deg = cpp_in.min_tilt_roll_deg;
    c_out.max_tilt_yaw_deg = cpp_in.max_tilt_yaw_deg;
    c_out.max_tilt_pitch_deg = cpp_in.max_tilt_pitch_deg;
    c_out.max_tilt_roll_deg = cpp_in.max_tilt_roll_deg;
    Copy3(cpp_in.tilt_center, c_out.tilt_center);
    c_out.downwash_factor = cpp_in.downwash_factor;
    c_out.ccw = cpp_in.ccw ? 1 : 0;
    c_out.shared_flap_hinge = cpp_in.shared_flap_hinge ? 1 : 0;
    c_out.no_torque = cpp_in.no_torque ? 1 : 0;
    SetCString(cpp_in.name, c_out.name, sizeof(c_out.name));
    c_out.pitch_a_deg = cpp_in.pitch_a_deg;
    c_out.pitch_b_deg = cpp_in.pitch_b_deg;
    c_out.force_at_pitch_a_lb = cpp_in.force_at_pitch_a_lb;
    c_out.power_at_pitch_0_kw = cpp_in.power_at_pitch_0_kw;
    c_out.power_at_pitch_b_kw = cpp_in.power_at_pitch_b_kw;
    c_out.translift_ve = cpp_in.translift_ve;
    c_out.translift_maxfactor = cpp_in.translift_maxfactor;
    c_out.ground_effect_constant = cpp_in.ground_effect_constant;
    c_out.vortex_state_lift_factor = cpp_in.vortex_state_lift_factor;
    c_out.vortex_state_c1 = cpp_in.vortex_state_c1;
    c_out.vortex_state_c2 = cpp_in.vortex_state_c2;
    c_out.vortex_state_c3 = cpp_in.vortex_state_c3;
    c_out.vortex_state_e1 = cpp_in.vortex_state_e1;
    c_out.vortex_state_e2 = cpp_in.vortex_state_e2;
    c_out.twist_deg = cpp_in.twist_deg;
    c_out.number_of_segments = cpp_in.number_of_segments;
    c_out.number_of_parts = cpp_in.number_of_parts;
    c_out.rel_len_where_incidence_is_measured = cpp_in.rel_len_where_incidence_is_measured;
    c_out.chord = cpp_in.chord;
    c_out.taper = cpp_in.taper;
    c_out.airfoil_incidence_no_lift_deg = cpp_in.airfoil_incidence_no_lift_deg;
    c_out.rel_len_blade_start = cpp_in.rel_len_blade_start;
    c_out.incidence_stall_zero_speed_deg = cpp_in.incidence_stall_zero_speed_deg;
    c_out.incidence_stall_half_sonic_speed_deg = cpp_in.incidence_stall_half_sonic_speed_deg;
    c_out.lift_factor_stall = cpp_in.lift_factor_stall;
    c_out.stall_change_over_deg = cpp_in.stall_change_over_deg;
    c_out.drag_factor_stall = cpp_in.drag_factor_stall;
    c_out.airfoil_lift_coefficient = cpp_in.airfoil_lift_coefficient;
    c_out.airfoil_drag_coefficient0 = cpp_in.airfoil_drag_coefficient0;
    c_out.airfoil_drag_coefficient1 = cpp_in.airfoil_drag_coefficient1;
    c_out.cyclic_factor = cpp_in.cyclic_factor;
    c_out.rotor_correction_factor = cpp_in.rotor_correction_factor;
}

void ToCpp(const RotorGearConfig& c_in, rotor::RotorGearConfig& cpp_out)
{
    cpp_out.max_power_engine_kw = c_in.max_power_engine_kw;
    cpp_out.engine_prop_factor = c_in.engine_prop_factor;
    cpp_out.yasim_drag_factor = c_in.yasim_drag_factor;
    cpp_out.yasim_lift_factor = c_in.yasim_lift_factor;
    cpp_out.max_power_rotor_brake_kw = c_in.max_power_rotor_brake_kw;
    cpp_out.rotorgear_friction_kw = c_in.rotorgear_friction_kw;
    cpp_out.engine_accel_limit = c_in.engine_accel_limit;
    cpp_out.initial_rel_rpm = c_in.initial_rel_rpm;
}

void ToC(const rotor::RotorGearConfig& cpp_in, RotorGearConfig& c_out)
{
    c_out.max_power_engine_kw = cpp_in.max_power_engine_kw;
    c_out.engine_prop_factor = cpp_in.engine_prop_factor;
    c_out.yasim_drag_factor = cpp_in.yasim_drag_factor;
    c_out.yasim_lift_factor = cpp_in.yasim_lift_factor;
    c_out.max_power_rotor_brake_kw = cpp_in.max_power_rotor_brake_kw;
    c_out.rotorgear_friction_kw = cpp_in.rotorgear_friction_kw;
    c_out.engine_accel_limit = cpp_in.engine_accel_limit;
    c_out.initial_rel_rpm = cpp_in.initial_rel_rpm;
}

void ToCpp(const RotorState& c_in, yasim::State& cpp_out)
{
    Copy3d(c_in.pos, cpp_out.pos);
    Copy9(c_in.orient, cpp_out.orient);
    Copy3(c_in.v, cpp_out.v);
    Copy3(c_in.rot, cpp_out.rot);
    Zero3(cpp_out.acc);
    Zero3(cpp_out.racc);
}

void ToC(const rotor::RotorResult& cpp_in, RotorResult& c_out)
{
    Copy3(cpp_in.force, c_out.force);
    Copy3(cpp_in.moment, c_out.moment);
    Copy3(cpp_in.arm_moment, c_out.arm_moment);
    Copy3(cpp_in.direct_torque, c_out.direct_torque);
    c_out.scalar_torque = cpp_in.scalar_torque;
    c_out.omega_rel = cpp_in.omega_rel;
    c_out.omega_rel_next = cpp_in.omega_rel_next;
    c_out.rpm = cpp_in.rpm;
}

void ToCpp(const RotorStepInput& c_in, rotor::RotorInstanceInput& cpp_out)
{
    ToCpp(c_in.state, cpp_out.state);
    cpp_out.dt = c_in.dt;
    cpp_out.rho = c_in.rho;
    Copy3(c_in.wind_global, cpp_out.wind_global);
    Copy3(c_in.cg_local, cpp_out.cg_local);
    cpp_out.engine_on = c_in.engine_on != 0;
    cpp_out.rotor_brake = c_in.rotor_brake;
    cpp_out.max_rel_torque = c_in.max_rel_torque;
    cpp_out.rel_target = c_in.rel_target;
    cpp_out.ground_provider = 0;
}

void ToC(const rotor::RotorTransmissionOutput& cpp_in, RotorTransmissionOutput& c_out)
{
    Copy3(cpp_in.total_force, c_out.total_force);
    Copy3(cpp_in.total_moment, c_out.total_moment);
    Copy3(cpp_in.gear_torque, c_out.gear_torque);
    c_out.engine_torque = cpp_in.engine_torque;
}

void ToC(const rotor::RotorInstanceOutput& cpp_in, RotorOutput& c_out)
{
    Copy3(cpp_in.total_force, c_out.total_force);
    Copy3(cpp_in.total_moment, c_out.total_moment);
    Copy3(cpp_in.gear_torque, c_out.gear_torque);
    c_out.engine_torque = cpp_in.engine_torque;
    ToC(cpp_in.rotor, c_out.rotor);
}

class GroundProviderAdapter : public rotor::GroundProvider {
public:
    GroundProviderAdapter(RotorGroundPlaneFn fn, void* user_data)
        : _fn(fn), _user_data(user_data)
    {
    }

    virtual void GetGroundPlane(const double pos[3], double plane[4], float vel[3]) const
    {
        if(_fn != 0) {
            _fn(_user_data, pos, plane, vel);
        }
    }

private:
    RotorGroundPlaneFn _fn;
    void* _user_data;
};

} // namespace

struct RotorContext {
    RotorContextImpl impl;
};

extern "C" {

RotorContext* Rotor_CreateContext(void)
{
    return new RotorContext();
}

void Rotor_DestroyContext(RotorContext* context)
{
    delete context;
}

void Rotor_InitRotorConfig(RotorConfig* out_config)
{
    if(out_config == 0) {
        return;
    }
    rotor::RotorConfig cpp_default;
    ToC(cpp_default, *out_config);
}

void Rotor_InitGearConfig(RotorGearConfig* out_config)
{
    if(out_config == 0) {
        return;
    }
    rotor::RotorGearConfig cpp_default;
    ToC(cpp_default, *out_config);
}

void Rotor_InitControlInput(RotorControlInput* out_input)
{
    if(out_input == 0) {
        return;
    }
    rotor::RotorControlInput cpp_default;
    ToC(cpp_default, *out_input);
}

void Rotor_InitStepInput(RotorStepInput* out_input)
{
    if(out_input == 0) {
        return;
    }
    Zero3d(out_input->state.pos);
    Zero3(out_input->state.v);
    Zero3(out_input->state.rot);
    int i;
    for(i = 0; i < 9; ++i) {
        out_input->state.orient[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    }
    out_input->dt = 1.0f / 60.0f;
    out_input->rho = 1.184f;
    Zero3(out_input->wind_global);
    Zero3(out_input->cg_local);
    out_input->engine_on = 1;
    out_input->rotor_brake = 0.0f;
    out_input->max_rel_torque = 1.0f;
    out_input->rel_target = 1.0f;
    out_input->ground_provider.get_ground_plane = 0;
    out_input->ground_provider.user_data = 0;
}

void Rotor_InitTransmissionOutput(RotorTransmissionOutput* out_output)
{
    if(out_output == 0) {
        return;
    }
    std::memset(out_output, 0, sizeof(*out_output));
}

void Rotor_InitOutput(RotorOutput* out_output)
{
    if(out_output == 0) {
        return;
    }
    std::memset(out_output, 0, sizeof(*out_output));
}

int Rotor_CreateTransmission(RotorContext* context, const RotorGearConfig* gear_config)
{
    if(context == 0 || gear_config == 0 || !ValidateGearConfig(*gear_config)) {
        return -1;
    }
    rotor::RotorGearConfig cpp_config;
    ToCpp(*gear_config, cpp_config);
    return context->impl.manager.CreateTransmission(cpp_config);
}

int Rotor_CreateRotor(RotorContext* context, const RotorConfig* rotor_config)
{
    if(context == 0 || rotor_config == 0 || !ValidateRotorConfig(*rotor_config)) {
        return -1;
    }
    rotor::RotorConfig cpp_config;
    ToCpp(*rotor_config, cpp_config);
    return context->impl.manager.CreateRotor(cpp_config);
}

int Rotor_AttachRotor(RotorContext* context, int transmission_idx, int rotor_idx)
{
    if(context == 0) {
        return 0;
    }
    return context->impl.manager.AttachRotor(transmission_idx, rotor_idx) ? 1 : 0;
}

int Rotor_GetTransmissionCount(const RotorContext* context)
{
    if(context == 0) {
        return 0;
    }
    return (int)context->impl.manager.GetTransmissionCount();
}

int Rotor_GetRotorCount(const RotorContext* context)
{
    if(context == 0) {
        return 0;
    }
    return (int)context->impl.manager.GetRotorCount();
}

int Rotor_SetControl(RotorContext* context, int rotor_idx, const RotorControlInput* control)
{
    if(context == 0 || control == 0 || !ValidateControlInput(*control)) {
        return 0;
    }
    rotor::RotorControlInput cpp_control;
    ToCpp(*control, cpp_control);
    return context->impl.manager.SetRotorControl(rotor_idx, cpp_control) ? 1 : 0;
}

int Rotor_ResetTransmission(RotorContext* context, int transmission_idx, float initial_rel_rpm)
{
    if(context == 0 || !IsFiniteFloat(initial_rel_rpm) || initial_rel_rpm < 0.0f) {
        return 0;
    }
    return context->impl.manager.ResetTransmission(transmission_idx, initial_rel_rpm) ? 1 : 0;
}

int Rotor_StepTransmission(RotorContext* context, int transmission_idx, const RotorStepInput* input)
{
    if(context == 0 || input == 0 || !ValidateStepInput(*input)) {
        return 0;
    }

    rotor::RotorInstanceInput cpp_input;
    ToCpp(*input, cpp_input);
    GroundProviderAdapter adapter(input->ground_provider.get_ground_plane,
        input->ground_provider.user_data);
    if(input->ground_provider.get_ground_plane != 0) {
        cpp_input.ground_provider = &adapter;
    } else {
        cpp_input.ground_provider = 0;
    }

    return context->impl.manager.StepTransmission(transmission_idx, cpp_input) ? 1 : 0;
}

int Rotor_GetTransmissionOutput(const RotorContext* context, int transmission_idx,
    RotorTransmissionOutput* output)
{
    if(context == 0 || output == 0) {
        return 0;
    }

    rotor::RotorTransmissionOutput cpp_output;
    if(!context->impl.manager.GetTransmissionOutput(transmission_idx, cpp_output)) {
        return 0;
    }
    ToC(cpp_output, *output);
    return 1;
}

int Rotor_GetRotorOutput(const RotorContext* context, int rotor_idx, RotorOutput* output)
{
    if(context == 0 || output == 0) {
        return 0;
    }

    rotor::RotorInstanceOutput cpp_output;
    if(!context->impl.manager.GetRotorOutput(rotor_idx, cpp_output)) {
        return 0;
    }
    ToC(cpp_output, *output);
    return 1;
}

void Rotor_Clear(RotorContext* context)
{
    if(context == 0) {
        return;
    }
    context->impl.manager.Clear();
}

} // extern "C"
