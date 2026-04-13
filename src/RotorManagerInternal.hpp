#ifndef ROTOR_MANAGER_INTERNAL_HPP
#define ROTOR_MANAGER_INTERNAL_HPP

#include <cstddef>

#include "RotorSystemInternal.hpp"

namespace rotor {

struct RotorCreateInput {
    RotorConfig rotor;
    RotorGearConfig gear;

    RotorCreateInput();
};

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
    RotorControlInput control;

    RotorInstanceInput();
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

    int CreateRotor(const RotorCreateInput& create_input);
    int AddRotor(const RotorConfig& rotor_config, const RotorGearConfig& gear_config);
    void Clear();
    std::size_t GetRotorCount() const;

    bool SetRotorControl(int idx, const RotorControlInput& control);
    bool ResetRotor(int idx, float initial_rel_rpm);
    bool StepRotor(int idx, const RotorInstanceInput& input);
    bool StepRotor(int idx, const RotorInstanceInput& input, RotorInstanceOutput& output);
    bool GetRotorOutput(int idx, RotorInstanceOutput& output) const;

private:
    RotorManager(const RotorManager&);
    RotorManager& operator=(const RotorManager&);

    class Impl;
    Impl* _impl;
};

} // namespace rotor

#endif
