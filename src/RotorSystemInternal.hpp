#ifndef ROTOR_SYSTEM_INTERNAL_HPP
#define ROTOR_SYSTEM_INTERNAL_HPP

#include <cstddef>
#include <string>
#include <vector>

#include "RotorState.hpp"

namespace rotor {

class GroundProvider {
public:
    virtual ~GroundProvider() {}
    virtual void GetGroundPlane(const double pos[3], double plane[4], float vel[3]) const = 0;
};

struct RotorControlInput {
    float collective;
    float cyclic_ail;
    float cyclic_ele;
    float tilt_roll;
    float tilt_pitch;
    float tilt_yaw;
    float balance;

    RotorControlInput();
};

struct RotorConfig {
    float base[3];
    float normal[3];
    float forward[3];

    float max_cyclic_ail_deg;
    float max_cyclic_ele_deg;
    float min_cyclic_ail_deg;
    float min_cyclic_ele_deg;
    float max_collective_deg;
    float min_collective_deg;
    float diameter_m;
    float weight_per_blade_lb;
    float num_blades;
    float rel_blade_center;
    float dynamic;
    float delta3;
    float delta;
    float translift;
    float drag_factor;
    float steps_per_second;
    float phi0_rad;
    float rpm;
    float rel_len_flap_hinge;
    float flap0_rad;
    float flapmin_rad;
    float flapmax_rad;
    float flap0_factor;
    float teeter_damp;
    float max_teeter_damp;
    float rel_len_teeter_hinge;
    float balance;
    float min_tilt_yaw_deg;
    float min_tilt_pitch_deg;
    float min_tilt_roll_deg;
    float max_tilt_yaw_deg;
    float max_tilt_pitch_deg;
    float max_tilt_roll_deg;
    float tilt_center[3];
    float downwash_factor;

    bool ccw;
    bool shared_flap_hinge;
    bool no_torque;

    std::string name;

    float pitch_a_deg;
    float pitch_b_deg;
    float force_at_pitch_a_lb;
    float power_at_pitch_0_kw;
    float power_at_pitch_b_kw;

    float translift_ve;
    float translift_maxfactor;
    float ground_effect_constant;
    float vortex_state_lift_factor;
    float vortex_state_c1;
    float vortex_state_c2;
    float vortex_state_c3;
    float vortex_state_e1;
    float vortex_state_e2;
    float twist_deg;
    float number_of_segments;
    float number_of_parts;
    float rel_len_where_incidence_is_measured;
    float chord;
    float taper;
    float airfoil_incidence_no_lift_deg;
    float rel_len_blade_start;
    float incidence_stall_zero_speed_deg;
    float incidence_stall_half_sonic_speed_deg;
    float lift_factor_stall;
    float stall_change_over_deg;
    float drag_factor_stall;
    float airfoil_lift_coefficient;
    float airfoil_drag_coefficient0;
    float airfoil_drag_coefficient1;
    float cyclic_factor;
    float rotor_correction_factor;

    RotorConfig();
};

struct RotorGearConfig {
    float max_power_engine_kw;
    float engine_prop_factor;
    float yasim_drag_factor;
    float yasim_lift_factor;
    float max_power_rotor_brake_kw;
    float rotorgear_friction_kw;
    float engine_accel_limit;
    float initial_rel_rpm;

    RotorGearConfig();
};

struct RotorSystemConfig {
    RotorGearConfig gear;
    std::vector<RotorConfig> rotors;
};

struct StepInput {
    yasim::State state;
    float dt;
    float rho;
    float wind_global[3];
    float cg_local[3];
    bool engine_on;
    float rotor_brake;
    float max_rel_torque;
    float rel_target;
    const GroundProvider* ground_provider;
    std::vector<RotorControlInput> rotor_controls;

    StepInput();
};

struct RotorResult {
    float force[3];
    float moment[3];
    float arm_moment[3];
    float direct_torque[3];
    float scalar_torque;
    float omega_rel;
    float omega_rel_next;
    float rpm;

    RotorResult();
};

struct StepOutput {
    float total_force[3];
    float total_moment[3];
    float gear_torque[3];
    float engine_torque;
    std::vector<RotorResult> rotors;

    StepOutput();
    void Resize(std::size_t rotor_count);
};

class RotorSystem {
public:
    RotorSystem();
    ~RotorSystem();

    bool Initialize(const RotorSystemConfig& config);
    void Reset(float initial_rel_rpm);
    bool Step(const StepInput& input, StepOutput& output);
    std::size_t GetRotorCount() const;

private:
    RotorSystem(const RotorSystem&);
    RotorSystem& operator=(const RotorSystem&);

    class Impl;
    Impl* _impl;
};

} // namespace rotor

#endif
