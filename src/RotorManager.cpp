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

RotorTransmissionOutput::RotorTransmissionOutput()
    : engine_torque(0.0f)
{
    Zero3(total_force);
    Zero3(total_moment);
    Zero3(gear_torque);
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
    struct RotorSlot {
        RotorSlot()
            : transmission_idx(-1),
              has_output(false)
        {
        }

        RotorConfig config;
        RotorControlInput control;
        RotorInstanceOutput output;
        int transmission_idx;
        bool has_output;
    };

    struct TransmissionSlot {
        TransmissionSlot()
            : system(0),
              initial_rel_rpm(1.0f),
              dirty(true),
              has_output(false)
        {
        }

        ~TransmissionSlot()
        {
            delete system;
        }

        RotorGearConfig gear_config;
        RotorSystem* system;
        std::vector<int> rotor_indices;
        RotorTransmissionOutput output;
        float initial_rel_rpm;
        bool dirty;
        bool has_output;
    };

    ~Impl()
    {
        Clear();
    }

    void Clear()
    {
        std::size_t i;
        for(i = 0; i < transmissions.size(); ++i) {
            delete transmissions[i];
        }
        transmissions.clear();

        for(i = 0; i < rotors.size(); ++i) {
            delete rotors[i];
        }
        rotors.clear();
    }

    static void ClearRotorOutput(RotorSlot& rotor)
    {
        rotor.output = RotorInstanceOutput();
        rotor.has_output = false;
    }

    void ClearTransmissionOutputs(int transmission_idx)
    {
        TransmissionSlot& transmission = *transmissions[transmission_idx];
        transmission.output = RotorTransmissionOutput();
        transmission.has_output = false;

        std::size_t i;
        for(i = 0; i < transmission.rotor_indices.size(); ++i) {
            ClearRotorOutput(*rotors[transmission.rotor_indices[i]]);
        }
    }

    void MarkTransmissionDirty(int transmission_idx)
    {
        if(transmission_idx < 0 || (std::size_t)transmission_idx >= transmissions.size()) {
            return;
        }
        transmissions[transmission_idx]->dirty = true;
        ClearTransmissionOutputs(transmission_idx);
    }

    void RemoveRotorFromTransmission(int transmission_idx, int rotor_idx)
    {
        if(transmission_idx < 0 || (std::size_t)transmission_idx >= transmissions.size()) {
            return;
        }

        TransmissionSlot& transmission = *transmissions[transmission_idx];
        std::size_t i;
        for(i = 0; i < transmission.rotor_indices.size(); ++i) {
            if(transmission.rotor_indices[i] == rotor_idx) {
                transmission.rotor_indices.erase(transmission.rotor_indices.begin() + i);
                break;
            }
        }
        MarkTransmissionDirty(transmission_idx);
    }

    bool RebuildTransmission(int transmission_idx, float initial_rel_rpm)
    {
        if(transmission_idx < 0 || (std::size_t)transmission_idx >= transmissions.size()) {
            return false;
        }

        TransmissionSlot& transmission = *transmissions[transmission_idx];
        transmission.initial_rel_rpm = initial_rel_rpm;

        delete transmission.system;
        transmission.system = 0;

        if(transmission.rotor_indices.empty()) {
            transmission.dirty = false;
            transmission.has_output = false;
            transmission.output = RotorTransmissionOutput();
            return true;
        }

        RotorSystemConfig config;
        config.gear = transmission.gear_config;
        config.gear.initial_rel_rpm = initial_rel_rpm;

        std::size_t i;
        for(i = 0; i < transmission.rotor_indices.size(); ++i) {
            config.rotors.push_back(rotors[transmission.rotor_indices[i]]->config);
        }

        RotorSystem* system = new RotorSystem();
        if(!system->Initialize(config)) {
            delete system;
            return false;
        }

        transmission.system = system;
        transmission.dirty = false;
        transmission.output = RotorTransmissionOutput();
        transmission.has_output = false;

        for(i = 0; i < transmission.rotor_indices.size(); ++i) {
            ClearRotorOutput(*rotors[transmission.rotor_indices[i]]);
        }

        return true;
    }

    std::vector<TransmissionSlot*> transmissions;
    std::vector<RotorSlot*> rotors;
};

RotorManager::RotorManager()
    : _impl(new Impl())
{
}

RotorManager::~RotorManager()
{
    delete _impl;
}

int RotorManager::CreateTransmission(const RotorGearConfig& gear_config)
{
    Impl::TransmissionSlot* transmission = new Impl::TransmissionSlot();
    transmission->gear_config = gear_config;
    transmission->initial_rel_rpm = gear_config.initial_rel_rpm;
    _impl->transmissions.push_back(transmission);
    return (int)(_impl->transmissions.size() - 1);
}

int RotorManager::CreateRotor(const RotorConfig& rotor_config)
{
    Impl::RotorSlot* rotor = new Impl::RotorSlot();
    rotor->config = rotor_config;
    _impl->rotors.push_back(rotor);
    return (int)(_impl->rotors.size() - 1);
}

void RotorManager::Clear()
{
    _impl->Clear();
}

std::size_t RotorManager::GetTransmissionCount() const
{
    return _impl->transmissions.size();
}

std::size_t RotorManager::GetRotorCount() const
{
    return _impl->rotors.size();
}

bool RotorManager::AttachRotor(int transmission_idx, int rotor_idx)
{
    if(transmission_idx < 0 || (std::size_t)transmission_idx >= _impl->transmissions.size()) {
        return false;
    }
    if(rotor_idx < 0 || (std::size_t)rotor_idx >= _impl->rotors.size()) {
        return false;
    }

    Impl::RotorSlot& rotor = *_impl->rotors[rotor_idx];
    if(rotor.transmission_idx == transmission_idx) {
        return true;
    }

    if(rotor.transmission_idx >= 0) {
        _impl->RemoveRotorFromTransmission(rotor.transmission_idx, rotor_idx);
    }

    _impl->transmissions[transmission_idx]->rotor_indices.push_back(rotor_idx);
    rotor.transmission_idx = transmission_idx;
    _impl->MarkTransmissionDirty(transmission_idx);
    return true;
}

bool RotorManager::SetRotorControl(int rotor_idx, const RotorControlInput& control)
{
    if(rotor_idx < 0 || (std::size_t)rotor_idx >= _impl->rotors.size()) {
        return false;
    }

    _impl->rotors[rotor_idx]->control = control;
    return true;
}

bool RotorManager::ResetTransmission(int transmission_idx, float initial_rel_rpm)
{
    if(transmission_idx < 0 || (std::size_t)transmission_idx >= _impl->transmissions.size()) {
        return false;
    }

    return _impl->RebuildTransmission(transmission_idx, initial_rel_rpm);
}

bool RotorManager::StepTransmission(int transmission_idx, const RotorInstanceInput& input)
{
    if(transmission_idx < 0 || (std::size_t)transmission_idx >= _impl->transmissions.size()) {
        return false;
    }

    Impl::TransmissionSlot& transmission = *_impl->transmissions[transmission_idx];
    if(transmission.dirty || transmission.system == 0) {
        if(!_impl->RebuildTransmission(transmission_idx, transmission.initial_rel_rpm)) {
            return false;
        }
    }

    if(transmission.system == 0) {
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
    sys_input.rotor_controls.resize(transmission.rotor_indices.size());

    std::size_t i;
    for(i = 0; i < transmission.rotor_indices.size(); ++i) {
        sys_input.rotor_controls[i] = _impl->rotors[transmission.rotor_indices[i]]->control;
    }

    StepOutput sys_output;
    if(!transmission.system->Step(sys_input, sys_output)) {
        return false;
    }

    Copy3(sys_output.total_force, transmission.output.total_force);
    Copy3(sys_output.total_moment, transmission.output.total_moment);
    Copy3(sys_output.gear_torque, transmission.output.gear_torque);
    transmission.output.engine_torque = sys_output.engine_torque;
    transmission.has_output = true;

    for(i = 0; i < transmission.rotor_indices.size(); ++i) {
        Impl::RotorSlot& rotor = *_impl->rotors[transmission.rotor_indices[i]];
        Copy3(sys_output.total_force, rotor.output.total_force);
        Copy3(sys_output.total_moment, rotor.output.total_moment);
        Copy3(sys_output.gear_torque, rotor.output.gear_torque);
        rotor.output.engine_torque = sys_output.engine_torque;
        if(i < sys_output.rotors.size()) {
            rotor.output.rotor = sys_output.rotors[i];
        } else {
            rotor.output.rotor = RotorResult();
        }
        rotor.has_output = true;
    }

    return true;
}

bool RotorManager::StepTransmission(int transmission_idx, const RotorInstanceInput& input,
    RotorTransmissionOutput& output)
{
    if(!StepTransmission(transmission_idx, input)) {
        return false;
    }
    return GetTransmissionOutput(transmission_idx, output);
}

bool RotorManager::GetTransmissionOutput(int transmission_idx,
    RotorTransmissionOutput& output) const
{
    if(transmission_idx < 0 || (std::size_t)transmission_idx >= _impl->transmissions.size()) {
        return false;
    }

    const Impl::TransmissionSlot& transmission = *_impl->transmissions[transmission_idx];
    if(!transmission.has_output) {
        return false;
    }

    output = transmission.output;
    return true;
}

bool RotorManager::GetRotorOutput(int rotor_idx, RotorInstanceOutput& output) const
{
    if(rotor_idx < 0 || (std::size_t)rotor_idx >= _impl->rotors.size()) {
        return false;
    }

    const Impl::RotorSlot& rotor = *_impl->rotors[rotor_idx];
    if(!rotor.has_output) {
        return false;
    }

    output = rotor.output;
    return true;
}

} // namespace rotor
