#include "RotorManagerInternal.hpp"

#include <vector>

namespace {

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

} // namespace

namespace rotor {

RotorCreateInput::RotorCreateInput()
{
}

RotorInstanceInput::RotorInstanceInput()
    : dt(1.0f / 60.0f),
      rho(1.184f),
      engine_on(true),
      rotor_brake(0.0f),
      max_rel_torque(1.0f),
      rel_target(1.0f),
      ground_provider(0)
{
    Zero3(wind_global);
    Zero3(cg_local);
}

RotorInstanceOutput::RotorInstanceOutput()
    : engine_torque(0.0f)
{
    Zero3(total_force);
    Zero3(total_moment);
    Zero3(gear_torque);
}

class RotorManager::Impl {
public:
    ~Impl()
    {
        Clear();
    }

    void Clear()
    {
        std::size_t i;
        for(i = 0; i < systems.size(); ++i) {
            delete systems[i];
        }
        systems.clear();
        controls.clear();
        outputs.clear();
    }

    std::vector<RotorSystem*> systems;
    std::vector<RotorControlInput> controls;
    std::vector<RotorInstanceOutput> outputs;
};

RotorManager::RotorManager()
    : _impl(new Impl())
{
}

RotorManager::~RotorManager()
{
    delete _impl;
}

int RotorManager::CreateRotor(const RotorCreateInput& create_input)
{
    RotorSystemConfig config;
    config.gear = create_input.gear;
    config.rotors.push_back(create_input.rotor);

    RotorSystem* system = new RotorSystem();
    if(!system->Initialize(config)) {
        delete system;
        return -1;
    }

    _impl->systems.push_back(system);
    _impl->controls.push_back(RotorControlInput());
    _impl->outputs.push_back(RotorInstanceOutput());
    return (int)(_impl->systems.size() - 1);
}

int RotorManager::AddRotor(const RotorConfig& rotor_config,
    const RotorGearConfig& gear_config)
{
    RotorCreateInput create_input;
    create_input.rotor = rotor_config;
    create_input.gear = gear_config;
    return CreateRotor(create_input);
}

void RotorManager::Clear()
{
    _impl->Clear();
}

std::size_t RotorManager::GetRotorCount() const
{
    return _impl->systems.size();
}

bool RotorManager::SetRotorControl(int idx, const RotorControlInput& control)
{
    if(idx < 0 || (std::size_t)idx >= _impl->systems.size()) {
        return false;
    }

    _impl->controls[idx] = control;
    return true;
}

bool RotorManager::ResetRotor(int idx, float initial_rel_rpm)
{
    if(idx < 0 || (std::size_t)idx >= _impl->systems.size()) {
        return false;
    }

    _impl->systems[idx]->Reset(initial_rel_rpm);
    return true;
}

bool RotorManager::StepRotor(int idx, const RotorInstanceInput& input)
{
    if(idx < 0 || (std::size_t)idx >= _impl->systems.size()) {
        return false;
    }

    StepInput sys_input;
    sys_input.state = input.state;
    sys_input.dt = input.dt;
    sys_input.rho = input.rho;
    Copy3(input.wind_global, sys_input.wind_global);
    Copy3(input.cg_local, sys_input.cg_local);
    sys_input.engine_on = input.engine_on;
    sys_input.rotor_brake = input.rotor_brake;
    sys_input.max_rel_torque = input.max_rel_torque;
    sys_input.rel_target = input.rel_target;
    sys_input.ground_provider = input.ground_provider;
    sys_input.rotor_controls.resize(1);
    sys_input.rotor_controls[0] = _impl->controls[idx];

    StepOutput sys_output;
    if(!_impl->systems[idx]->Step(sys_input, sys_output)) {
        return false;
    }

    RotorInstanceOutput& output = _impl->outputs[idx];
    Copy3(sys_output.total_force, output.total_force);
    Copy3(sys_output.total_moment, output.total_moment);
    Copy3(sys_output.gear_torque, output.gear_torque);
    output.engine_torque = sys_output.engine_torque;
    if(!sys_output.rotors.empty()) {
        output.rotor = sys_output.rotors[0];
    }

    return true;
}

bool RotorManager::StepRotor(int idx, const RotorInstanceInput& input,
    RotorInstanceOutput& output)
{
    if(!StepRotor(idx, input)) {
        return false;
    }

    return GetRotorOutput(idx, output);
}

bool RotorManager::GetRotorOutput(int idx, RotorInstanceOutput& output) const
{
    if(idx < 0 || (std::size_t)idx >= _impl->systems.size()) {
        return false;
    }

    output = _impl->outputs[idx];
    return true;
}

} // namespace rotor
