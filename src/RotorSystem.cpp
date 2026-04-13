#include "RotorSystemInternal.hpp"

#include "Ground.hpp"
#include "Glue.hpp"
#include "Math.hpp"
#include "Rotor.hpp"
#include "Rotorpart.hpp"

namespace {

const float kPi = 3.14159f;

void Zero3(float* v)
{
    v[0] = v[1] = v[2] = 0.0f;
}

void Copy3(const float* src, float* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

void Copy4(const double* src, double* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

void Accumulate3(const float* add, float* dst)
{
    dst[0] += add[0];
    dst[1] += add[1];
    dst[2] += add[2];
}

void PointVelocity(const float* pos, const float* cg, float* rot, float* out)
{
    float rel[3];
    rel[0] = pos[0] - cg[0];
    rel[1] = pos[1] - cg[1];
    rel[2] = pos[2] - cg[2];
    yasim::Math::cross3(rot, rel, out);
}

void AddForceAtPoint(const float* pos, const float* cg, const float* force,
    float* total_force, float* total_moment, float* arm_moment)
{
    float cg_mut[3];
    float pos_mut[3];
    float force_mut[3];
    float v[3];
    float t[3];

    Copy3(cg, cg_mut);
    Copy3(pos, pos_mut);
    Copy3(force, force_mut);

    Accumulate3(force_mut, total_force);
    yasim::Math::sub3(cg_mut, pos_mut, v);
    yasim::Math::cross3(force_mut, v, t);
    Accumulate3(t, total_moment);
    if(arm_moment != 0) {
        Accumulate3(t, arm_moment);
    }
}

void AddTorque(const float* torque, float* total_moment, float* direct_torque)
{
    Accumulate3(torque, total_moment);
    if(direct_torque != 0) {
        Accumulate3(torque, direct_torque);
    }
}

class ProviderGroundAdapter : public yasim::Ground {
public:
    explicit ProviderGroundAdapter(const rotor::GroundProvider* provider)
        : _provider(provider)
    {
    }

    virtual void getGroundPlane(const double pos[3], double plane[4], float vel[3])
    {
        if(_provider != 0) {
            _provider->GetGroundPlane(pos, plane, vel);
        } else {
            yasim::Ground::getGroundPlane(pos, plane, vel);
        }
    }

private:
    const rotor::GroundProvider* _provider;
};

void ApplyRotorConfig(yasim::Rotor* rotor, const rotor::RotorConfig& cfg)
{
    rotor->setBase((float*)cfg.base);
    rotor->setNormal((float*)cfg.normal);
    rotor->setForward((float*)cfg.forward);
    rotor->setMaxCyclicail(cfg.max_cyclic_ail_deg);
    rotor->setMaxCyclicele(cfg.max_cyclic_ele_deg);
    rotor->setMinCyclicail(cfg.min_cyclic_ail_deg);
    rotor->setMinCyclicele(cfg.min_cyclic_ele_deg);
    rotor->setMaxCollective(cfg.max_collective_deg);
    rotor->setMinCollective(cfg.min_collective_deg);
    rotor->setDiameter(cfg.diameter_m);
    rotor->setWeightPerBlade(cfg.weight_per_blade_lb);
    rotor->setNumberOfBlades(cfg.num_blades);
    rotor->setRelBladeCenter(cfg.rel_blade_center);
    rotor->setDynamic(cfg.dynamic);
    rotor->setDelta3(cfg.delta3);
    rotor->setDelta(cfg.delta);
    rotor->setTranslift(cfg.translift);
    rotor->setC2(cfg.drag_factor);
    rotor->setStepspersecond(cfg.steps_per_second);
    rotor->setPhiNull(cfg.phi0_rad);
    rotor->setRPM(cfg.rpm);
    rotor->setRelLenHinge(cfg.rel_len_flap_hinge);
    rotor->setAlpha0(cfg.flap0_rad);
    rotor->setAlphamin(cfg.flapmin_rad);
    rotor->setAlphamax(cfg.flapmax_rad);
    rotor->setAlpha0factor(cfg.flap0_factor);
    rotor->setTeeterdamp(cfg.teeter_damp);
    rotor->setMaxteeterdamp(cfg.max_teeter_damp);
    rotor->setRelLenTeeterHinge(cfg.rel_len_teeter_hinge);
    rotor->setBalance(cfg.balance);
    rotor->setMinTiltYaw(cfg.min_tilt_yaw_deg);
    rotor->setMinTiltPitch(cfg.min_tilt_pitch_deg);
    rotor->setMinTiltRoll(cfg.min_tilt_roll_deg);
    rotor->setMaxTiltYaw(cfg.max_tilt_yaw_deg);
    rotor->setMaxTiltPitch(cfg.max_tilt_pitch_deg);
    rotor->setMaxTiltRoll(cfg.max_tilt_roll_deg);
    rotor->setTiltCenterX(cfg.tilt_center[0]);
    rotor->setTiltCenterY(cfg.tilt_center[1]);
    rotor->setTiltCenterZ(cfg.tilt_center[2]);
    rotor->setDownwashFactor(cfg.downwash_factor);

    if(cfg.ccw) {
        rotor->setCcw(1);
    }
    if(cfg.shared_flap_hinge) {
        rotor->setSharedFlapHinge(true);
    }
    if(!cfg.name.empty()) {
        rotor->setName(cfg.name.c_str());
    }

    rotor->setPitchA(cfg.pitch_a_deg);
    rotor->setPitchB(cfg.pitch_b_deg);
    rotor->setForceAtPitchA(cfg.force_at_pitch_a_lb);
    rotor->setPowerAtPitch0(cfg.power_at_pitch_0_kw);
    rotor->setPowerAtPitchB(cfg.power_at_pitch_b_kw);
    if(cfg.no_torque) {
        rotor->setNotorque(1);
    }

    rotor->setParameter("translift_ve", cfg.translift_ve);
    rotor->setParameter("translift_maxfactor", cfg.translift_maxfactor);
    rotor->setParameter("ground_effect_constant", cfg.ground_effect_constant);
    rotor->setParameter("vortex_state_lift_factor", cfg.vortex_state_lift_factor);
    rotor->setParameter("vortex_state_c1", cfg.vortex_state_c1);
    rotor->setParameter("vortex_state_c2", cfg.vortex_state_c2);
    rotor->setParameter("vortex_state_c3", cfg.vortex_state_c3);
    rotor->setParameter("vortex_state_e1", cfg.vortex_state_e1);
    rotor->setParameter("vortex_state_e2", cfg.vortex_state_e2);
    rotor->setParameter("twist", cfg.twist_deg);
    rotor->setParameter("number_of_segments", cfg.number_of_segments);
    rotor->setParameter("number_of_parts", cfg.number_of_parts);
    rotor->setParameter("rel_len_where_incidence_is_measured",
        cfg.rel_len_where_incidence_is_measured);
    rotor->setParameter("chord", cfg.chord);
    rotor->setParameter("taper", cfg.taper);
    rotor->setParameter("airfoil_incidence_no_lift", cfg.airfoil_incidence_no_lift_deg);
    rotor->setParameter("rel_len_blade_start", cfg.rel_len_blade_start);
    rotor->setParameter("incidence_stall_zero_speed", cfg.incidence_stall_zero_speed_deg);
    rotor->setParameter("incidence_stall_half_sonic_speed",
        cfg.incidence_stall_half_sonic_speed_deg);
    rotor->setParameter("lift_factor_stall", cfg.lift_factor_stall);
    rotor->setParameter("stall_change_over", cfg.stall_change_over_deg);
    rotor->setParameter("drag_factor_stall", cfg.drag_factor_stall);
    rotor->setParameter("airfoil_lift_coefficient", cfg.airfoil_lift_coefficient);
    rotor->setParameter("airfoil_drag_coefficient0", cfg.airfoil_drag_coefficient0);
    rotor->setParameter("airfoil_drag_coefficient1", cfg.airfoil_drag_coefficient1);
    rotor->setParameter("cyclic_factor", cfg.cyclic_factor);
    rotor->setParameter("rotor_correction_factor", cfg.rotor_correction_factor);
}

void ComputeLocalWind(const float* pos, const yasim::State& state,
    const float* wind_global, const float* cg_local, yasim::Rotorgear* gear,
    float* out, bool is_rotor)
{
    float wind_mut[3];
    float lwind[3];
    float lrot[3];
    float lv[3];
    float cg_mut[3];
    float tmp[3];

    Copy3(wind_global, wind_mut);
    Copy3(cg_local, cg_mut);
    yasim::Math::vmul33((float*)state.orient, wind_mut, lwind);
    yasim::Math::vmul33((float*)state.orient, (float*)state.rot, lrot);
    yasim::Math::vmul33((float*)state.orient, (float*)state.v, lv);

    PointVelocity(pos, cg_mut, lrot, out);
    yasim::Math::mul3(-1.0f, out, out);
    yasim::Math::add3(lwind, out, out);
    yasim::Math::sub3(out, lv, out);

    if(!is_rotor && gear != 0 && gear->isInUse()) {
        gear->getDownWash((float*)pos, lv, tmp);
        yasim::Math::add3(out, tmp, out);
    }
}

} // namespace

namespace rotor {

RotorControlInput::RotorControlInput()
    : collective(0.0f),
      cyclic_ail(0.0f),
      cyclic_ele(0.0f),
      tilt_roll(0.0f),
      tilt_pitch(0.0f),
      tilt_yaw(0.0f),
      balance(1.0f)
{
}

RotorConfig::RotorConfig()
    : max_cyclic_ail_deg(7.6f),
      max_cyclic_ele_deg(4.94f),
      min_cyclic_ail_deg(-7.6f),
      min_cyclic_ele_deg(-4.94f),
      max_collective_deg(15.8f),
      min_collective_deg(-0.2f),
      diameter_m(10.2f),
      weight_per_blade_lb(44.0f),
      num_blades(4.0f),
      rel_blade_center(0.7f),
      dynamic(0.7f),
      delta3(0.0f),
      delta(0.0f),
      translift(0.05f),
      drag_factor(1.0f),
      steps_per_second(120.0f),
      phi0_rad(0.0f),
      rpm(424.0f),
      rel_len_flap_hinge(0.07f),
      flap0_rad(-5.0f * kPi / 180.0f),
      flapmin_rad(-15.0f * kPi / 180.0f),
      flapmax_rad(15.0f * kPi / 180.0f),
      flap0_factor(1.0f),
      teeter_damp(0.0001f),
      max_teeter_damp(1000.0f),
      rel_len_teeter_hinge(0.01f),
      balance(1.0f),
      min_tilt_yaw_deg(0.0f),
      min_tilt_pitch_deg(0.0f),
      min_tilt_roll_deg(0.0f),
      max_tilt_yaw_deg(0.0f),
      max_tilt_pitch_deg(0.0f),
      max_tilt_roll_deg(0.0f),
      downwash_factor(1.0f),
      ccw(false),
      shared_flap_hinge(false),
      no_torque(false),
      pitch_a_deg(10.0f),
      pitch_b_deg(10.0f),
      force_at_pitch_a_lb(3000.0f),
      power_at_pitch_0_kw(300.0f),
      power_at_pitch_b_kw(3000.0f),
      translift_ve(20.0f),
      translift_maxfactor(1.3f),
      ground_effect_constant(0.1f),
      vortex_state_lift_factor(0.4f),
      vortex_state_c1(0.1f),
      vortex_state_c2(0.0f),
      vortex_state_c3(0.0f),
      vortex_state_e1(1.0f),
      vortex_state_e2(1.0f),
      twist_deg(0.0f),
      number_of_segments(1.0f),
      number_of_parts(4.0f),
      rel_len_where_incidence_is_measured(0.7f),
      chord(0.3f),
      taper(1.0f),
      airfoil_incidence_no_lift_deg(0.0f),
      rel_len_blade_start(0.0f),
      incidence_stall_zero_speed_deg(18.0f),
      incidence_stall_half_sonic_speed_deg(14.0f),
      lift_factor_stall(0.28f),
      stall_change_over_deg(2.0f),
      drag_factor_stall(8.0f),
      airfoil_lift_coefficient(0.0f),
      airfoil_drag_coefficient0(0.0f),
      airfoil_drag_coefficient1(0.0f),
      cyclic_factor(1.0f),
      rotor_correction_factor(0.65f)
{
    base[0] = base[1] = base[2] = 0.0f;
    normal[0] = 0.0f;
    normal[1] = 0.0f;
    normal[2] = 1.0f;
    forward[0] = 1.0f;
    forward[1] = 0.0f;
    forward[2] = 0.0f;
    tilt_center[0] = tilt_center[1] = tilt_center[2] = 0.0f;
}

RotorGearConfig::RotorGearConfig()
    : max_power_engine_kw(450.0f),
      engine_prop_factor(0.05f),
      yasim_drag_factor(1.0f),
      yasim_lift_factor(1.0f),
      max_power_rotor_brake_kw(1.0f),
      rotorgear_friction_kw(1.0f),
      engine_accel_limit(0.05f),
      initial_rel_rpm(1.0f)
{
}

StepInput::StepInput()
    : dt(1.0f / 60.0f),
      rho(yasim::rho_null),
      engine_on(true),
      rotor_brake(0.0f),
      max_rel_torque(1.0f),
      rel_target(1.0f),
      ground_provider(0)
{
    wind_global[0] = wind_global[1] = wind_global[2] = 0.0f;
    cg_local[0] = cg_local[1] = cg_local[2] = 0.0f;
}

RotorResult::RotorResult()
    : scalar_torque(0.0f),
      omega_rel(0.0f),
      omega_rel_next(0.0f),
      rpm(0.0f)
{
    Zero3(force);
    Zero3(moment);
    Zero3(arm_moment);
    Zero3(direct_torque);
}

StepOutput::StepOutput()
    : engine_torque(0.0f)
{
    Zero3(total_force);
    Zero3(total_moment);
    Zero3(gear_torque);
}

void StepOutput::Resize(std::size_t rotor_count)
{
    rotors.assign(rotor_count, RotorResult());
    Zero3(total_force);
    Zero3(total_moment);
    Zero3(gear_torque);
    engine_torque = 0.0f;
}

class RotorSystem::Impl {
public:
    Impl()
        : initialized(false),
          initial_rel_rpm(1.0f),
          has_config(false)
    {
    }

    yasim::Rotorgear gear;
    std::vector<yasim::Rotor*> rotors;
    bool initialized;
    float initial_rel_rpm;
    RotorSystemConfig config;
    bool has_config;
};

RotorSystem::RotorSystem()
    : _impl(new Impl())
{
}

RotorSystem::~RotorSystem()
{
    delete _impl;
}

bool RotorSystem::Initialize(const RotorSystemConfig& config)
{
    delete _impl;
    _impl = new Impl();
    _impl->config = config;
    _impl->has_config = true;

    _impl->gear.setParameter((char*)"max_power_engine", config.gear.max_power_engine_kw);
    _impl->gear.setParameter((char*)"engine_prop_factor", config.gear.engine_prop_factor);
    _impl->gear.setParameter((char*)"yasimdragfactor", config.gear.yasim_drag_factor);
    _impl->gear.setParameter((char*)"yasimliftfactor", config.gear.yasim_lift_factor);
    _impl->gear.setParameter((char*)"max_power_rotor_brake", config.gear.max_power_rotor_brake_kw);
    _impl->gear.setParameter((char*)"rotorgear_friction", config.gear.rotorgear_friction_kw);
    _impl->gear.setParameter((char*)"engine_accel_limit", config.gear.engine_accel_limit);
    _impl->initial_rel_rpm = config.gear.initial_rel_rpm;

    std::size_t i;
    for(i = 0; i < config.rotors.size(); ++i) {
        yasim::Rotor* rotor = new yasim::Rotor();
        ApplyRotorConfig(rotor, config.rotors[i]);
        _impl->gear.addRotor(rotor);
        _impl->rotors.push_back(rotor);
    }

    if(_impl->rotors.empty()) {
        return false;
    }

    _impl->gear.compile();
    if(!_impl->rotors.empty()) {
        _impl->rotors[0]->setOmegaRelNeu(_impl->initial_rel_rpm);
    }
    _impl->initialized = true;
    return true;
}

void RotorSystem::Reset(float initial_rel_rpm)
{
    if(!_impl->has_config) {
        return;
    }

    RotorSystemConfig reset_config = _impl->config;
    reset_config.gear.initial_rel_rpm = initial_rel_rpm;
    Initialize(reset_config);
}

bool RotorSystem::Step(const StepInput& input, StepOutput& output)
{
    if(!_impl->initialized || _impl->rotors.empty()) {
        return false;
    }

    output.Resize(_impl->rotors.size());

    _impl->gear.setEngineOn(input.engine_on ? 1 : 0);
    _impl->gear.setRotorBrake(input.rotor_brake);
    _impl->gear.setRotorEngineMaxRelTorque(input.max_rel_torque);
    _impl->gear.setRotorRelTarget(input.rel_target);

    std::size_t i;
    for(i = 0; i < _impl->rotors.size(); ++i) {
        RotorControlInput control;
        if(i < input.rotor_controls.size()) {
            control = input.rotor_controls[i];
        }

        yasim::Rotor* rotor = _impl->rotors[i];
        rotor->setCollective(control.collective);
        rotor->setCyclicail(control.cyclic_ail, control.cyclic_ail);
        rotor->setCyclicele(control.cyclic_ele, control.cyclic_ele);
        rotor->setTiltRoll(control.tilt_roll);
        rotor->setTiltPitch(control.tilt_pitch);
        rotor->setTiltYaw(control.tilt_yaw);
        rotor->setRotorBalance(control.balance);
    }

    if(input.ground_provider != 0) {
        ProviderGroundAdapter ground(input.ground_provider);
        for(i = 0; i < _impl->rotors.size(); ++i) {
            _impl->rotors[i]->findGroundEffectAltitude(&ground, (yasim::State*)&input.state);
        }
    } else {
        for(i = 0; i < _impl->rotors.size(); ++i) {
            _impl->rotors[i]->setGroundEffectAltitude(1e9f);
        }
    }

    float lrot[3];
    yasim::Math::vmul33((float*)input.state.orient, (float*)input.state.rot, lrot);
    yasim::Math::mul3(input.dt, lrot, lrot);
    _impl->gear.initRotorIteration(lrot, input.dt);

    for(i = 0; i < _impl->rotors.size(); ++i) {
        yasim::Rotor* rotor = _impl->rotors[i];
        float rotor_hub_pos[3];
        float hub_wind[3];

        rotor->getPosition(rotor_hub_pos);
        ComputeLocalWind(rotor_hub_pos, input.state, input.wind_global, input.cg_local,
            &_impl->gear, hub_wind, false);
        rotor->calcLiftFactor(hub_wind, input.rho, (yasim::State*)&input.state);

        float scalar_total = 0.0f;
        int j;
        for(j = 0; j < rotor->numRotorparts(); ++j) {
            yasim::Rotorpart* part = rotor->getRotorpart(j);
            float part_pos[3];
            float part_force_pos[3];
            float part_wind[3];
            float force[3];
            float torque[3];
            float scalar_torque = 0.0f;

            part->getPosition(part_pos);
            ComputeLocalWind(part_pos, input.state, input.wind_global, input.cg_local,
                &_impl->gear, part_wind, true);
            part->calcForce(part_wind, input.rho, force, torque, &scalar_torque);

            part->getPositionForceAttac(part_force_pos);
            AddForceAtPoint(part_force_pos, input.cg_local, force,
                output.rotors[i].force, output.rotors[i].moment, output.rotors[i].arm_moment);
            AddForceAtPoint(part_force_pos, input.cg_local, force,
                output.total_force, output.total_moment, 0);
            AddTorque(torque, output.rotors[i].moment, output.rotors[i].direct_torque);
            AddTorque(torque, output.total_moment, 0);
            scalar_total += scalar_torque;
        }

        rotor->setTorque(scalar_total);
        output.rotors[i].scalar_torque = scalar_total;
        output.rotors[i].omega_rel = rotor->getOmegaRel();
        output.rotors[i].omega_rel_next = rotor->getOmegaRelNeu();
        output.rotors[i].rpm = rotor->getOmega() * 60.0f / (2.0f * kPi);
    }

    _impl->gear.calcForces(output.gear_torque);
    Accumulate3(output.gear_torque, output.total_moment);
    output.engine_torque = _impl->gear.getTotalTorqueOnEngine();

    for(i = 0; i < _impl->rotors.size(); ++i) {
        output.rotors[i].omega_rel = _impl->rotors[i]->getOmegaRel();
        output.rotors[i].omega_rel_next = _impl->rotors[i]->getOmegaRelNeu();
        output.rotors[i].rpm = _impl->rotors[i]->getOmega() * 60.0f / (2.0f * kPi);
    }

    return true;
}

std::size_t RotorSystem::GetRotorCount() const
{
    return _impl->rotors.size();
}

} // namespace rotor
