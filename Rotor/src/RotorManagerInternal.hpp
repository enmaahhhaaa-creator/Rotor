#ifndef ROTOR_MANAGER_INTERNAL_HPP
#define ROTOR_MANAGER_INTERNAL_HPP

#include <cstddef>

#include "RotorSystemInternal.hpp"

namespace rotor {

struct RotorInstanceInput {
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

    RotorInstanceInput();
};

struct RotorTransmissionOutput {
    float total_force[3];
    float total_moment[3];
    float gear_torque[3];
    float engine_torque;

    RotorTransmissionOutput();
};

struct RotorInstanceOutput {
    float total_force[3];
    float total_moment[3];
    float gear_torque[3];
    float engine_torque;
    RotorResult rotor;

    RotorInstanceOutput();
};

class RotorManager {
public:
    RotorManager();
    ~RotorManager();

    int CreateTransmission(const RotorGearConfig& gear_config);
    int CreateRotor(const RotorConfig& rotor_config);
    void Clear();
    std::size_t GetTransmissionCount() const;
    std::size_t GetRotorCount() const;

    bool AttachRotor(int transmission_idx, int rotor_idx);
    bool SetRotorControl(int rotor_idx, const RotorControlInput& control);
    bool ResetTransmission(int transmission_idx, float initial_rel_rpm);
    bool StepTransmission(int transmission_idx, const RotorInstanceInput& input);
    bool StepTransmission(int transmission_idx, const RotorInstanceInput& input,
        RotorTransmissionOutput& output);
    bool GetTransmissionOutput(int transmission_idx, RotorTransmissionOutput& output) const;
    bool GetRotorOutput(int rotor_idx, RotorInstanceOutput& output) const;

private:
    RotorManager(const RotorManager&);
    RotorManager& operator=(const RotorManager&);

    class Impl;
    Impl* _impl;
};

} // namespace rotor

#endif
