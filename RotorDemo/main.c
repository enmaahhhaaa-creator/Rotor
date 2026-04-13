#include <stdio.h>
#include <string.h>

#include "../include/Rotor.h"

static RotorConfig BuildMainRotorConfig(void)
{
    RotorConfig rotorConfig;

    /* 先用库内默认值填充一份旋翼配置。 */
    Rotor_InitRotorConfig(&rotorConfig);

    /* 配置旋翼名称，便于后续调试和日志识别。 */
    strncpy(rotorConfig.name, "main", sizeof(rotorConfig.name) - 1);
    rotorConfig.name[sizeof(rotorConfig.name) - 1] = '\0';

    /* 设置桨毂位置和旋翼坐标轴方向。 */
    rotorConfig.base[0] = 0.0f;
    rotorConfig.base[1] = 0.0f;
    rotorConfig.base[2] = 1.5f;
    rotorConfig.normal[0] = 0.0f;
    rotorConfig.normal[1] = 0.0f;
    rotorConfig.normal[2] = 1.0f;
    rotorConfig.forward[0] = 1.0f;
    rotorConfig.forward[1] = 0.0f;
    rotorConfig.forward[2] = 0.0f;

    /* 设置主旋翼的基础几何和动力学参数。 */
    rotorConfig.diameter_m = 10.2f;
    rotorConfig.weight_per_blade_lb = 44.0f;
    rotorConfig.num_blades = 4.0f;
    rotorConfig.rel_blade_center = 0.7f;
    rotorConfig.dynamic = 0.7f;
    rotorConfig.delta3 = 0.0f;
    rotorConfig.delta = 1.0f;
    rotorConfig.rpm = 424.0f;

    /* 设置总距和周期变距的控制范围。 */
    rotorConfig.max_collective_deg = 15.8f;
    rotorConfig.min_collective_deg = -0.2f;
    rotorConfig.max_cyclic_ail_deg = 7.6f;
    rotorConfig.max_cyclic_ele_deg = 4.94f;
    rotorConfig.min_cyclic_ail_deg = -7.6f;
    rotorConfig.min_cyclic_ele_deg = -4.94f;

    /* 设置用于气动拟合的参考点参数。 */
    rotorConfig.pitch_a_deg = 10.0f;
    rotorConfig.pitch_b_deg = 10.0f;
    rotorConfig.force_at_pitch_a_lb = 3000.0f;
    rotorConfig.power_at_pitch_0_kw = 300.0f;
    rotorConfig.power_at_pitch_b_kw = 3000.0f;

    /* 直接给出空气动力系数和离散化设置。 */
    rotorConfig.airfoil_lift_coefficient = 6.0f;
    rotorConfig.airfoil_drag_coefficient0 = 0.012f;
    rotorConfig.airfoil_drag_coefficient1 = 0.08f;
    rotorConfig.number_of_parts = 16.0f;
    rotorConfig.rotor_correction_factor = 0.65f;

    return rotorConfig;
}

static RotorGearConfig BuildMainTransmissionConfig(void)
{
    RotorGearConfig gearConfig;

    /* 先用库内默认值填充一份传动配置。 */
    Rotor_InitGearConfig(&gearConfig);

    /* 设置发动机和传动系统相关参数。 */
    gearConfig.max_power_engine_kw = 650.0f;
    gearConfig.engine_prop_factor = 0.05f;
    gearConfig.max_power_rotor_brake_kw = 20.0f;
    gearConfig.rotorgear_friction_kw = 1.0f;
    gearConfig.engine_accel_limit = 0.05f;

    return gearConfig;
}

static RotorControlInput BuildHoverControl(void)
{
    RotorControlInput rotorControl;

    /* 先生成一份中性操纵输入。 */
    Rotor_InitControlInput(&rotorControl);

    /* 构造一个适合悬停工况的控制量。 */
    rotorControl.collective = 0.35f;
    rotorControl.cyclic_ail = 0.0f;
    rotorControl.cyclic_ele = 0.0f;
    rotorControl.tilt_roll = 0.0f;
    rotorControl.tilt_pitch = 0.0f;
    rotorControl.tilt_yaw = 0.0f;
    rotorControl.balance = 1.0f;
    return rotorControl;
}

static RotorStepInput BuildBaseStepInput(void)
{
    RotorStepInput stepInput;

    /* 先生成一份基础步进输入。 */
    Rotor_InitStepInput(&stepInput);

    /* 设置一个近似海平面的初始全球位置。 */
    stepInput.state.pos[0] = 0.0;
    stepInput.state.pos[1] = 0.0;
    stepInput.state.pos[2] = 6378137.0;

    /* 设置环境密度和传动目标状态。 */
    stepInput.rho = 1.184f;
    stepInput.engine_on = 1;
    stepInput.rel_target = 1.0f;
    stepInput.max_rel_torque = 1.0f;
    return stepInput;
}

static void PrintVec3(const char* label, const float* v)
{
    /* 统一打印三维向量，减少重复格式字符串。 */
    printf("%s: [%.3f, %.3f, %.3f]\n", label, v[0], v[1], v[2]);
}

static void RunScenario(RotorContext* context, int transmissionIdx, int rotorIdx,
    const char* name, const RotorStepInput* stepInput)
{
    RotorTransmissionOutput transmissionOutput;
    RotorOutput rotorOutput;

    /* 每次场景求解前都把输出结构清零。 */
    Rotor_InitTransmissionOutput(&transmissionOutput);
    Rotor_InitOutput(&rotorOutput);

    /* 打印当前场景名，便于区分输出。 */
    printf("Scenario: %s\n", name);

    /* 先对整个传动系统执行一步求解。 */
    if(!Rotor_StepTransmission(context, transmissionIdx, stepInput)) {
        printf("  step failed\n");
        return;
    }

    /* 读取共享传动系统的总输出。 */
    if(!Rotor_GetTransmissionOutput(context, transmissionIdx, &transmissionOutput)) {
        printf("  transmission output failed\n");
        return;
    }

    /* 再读取目标旋翼自身的输出。 */
    if(!Rotor_GetRotorOutput(context, rotorIdx, &rotorOutput)) {
        printf("  rotor output failed\n");
        return;
    }

    /* 逐项打印 transmission 总输出和单个 rotor 输出。 */
    PrintVec3("  total_force_local", transmissionOutput.total_force);
    PrintVec3("  total_moment_local", transmissionOutput.total_moment);
    PrintVec3("  gear_torque_local", transmissionOutput.gear_torque);
    PrintVec3("  rotor_force_local", rotorOutput.rotor.force);
    PrintVec3("  rotor_moment_local", rotorOutput.rotor.moment);
    printf("  rotor_scalar_torque: %.3f\n", rotorOutput.rotor.scalar_torque);
    printf("  rotor_rpm: %.3f\n", rotorOutput.rotor.rpm);
    printf("  rotor_omega_rel(current/next): %.3f / %.3f\n",
        rotorOutput.rotor.omega_rel, rotorOutput.rotor.omega_rel_next);
    printf("  engine_torque: %.3f\n", transmissionOutput.engine_torque);
    printf("\n");
}

int main(void)
{
    RotorContext* context;
    RotorGearConfig mainTransmissionConfig;
    RotorConfig mainRotorConfig;
    int mainTransmissionIdx;
    int mainRotorIdx;
    RotorStepInput hoverStep;
    RotorControlInput hoverControl;
    RotorStepInput forwardFlightStep;
    RotorControlInput forwardFlightControl;
    int i;

    /* 创建旋翼上下文，并准备传动和旋翼的独立配置。 */
    context = Rotor_CreateContext();
    mainTransmissionConfig = BuildMainTransmissionConfig();
    mainRotorConfig = BuildMainRotorConfig();

    /* 先创建一套传动系统。 */
    mainTransmissionIdx = Rotor_CreateTransmission(context, &mainTransmissionConfig);
    if(mainTransmissionIdx < 0) {
        fprintf(stderr, "Failed to create transmission model.\n");
        Rotor_DestroyContext(context);
        return 1;
    }

    /* 再创建一个主旋翼实例。 */
    mainRotorIdx = Rotor_CreateRotor(context, &mainRotorConfig);
    if(mainRotorIdx < 0) {
        fprintf(stderr, "Failed to create main rotor model.\n");
        Rotor_DestroyContext(context);
        return 1;
    }

    /* 把主旋翼挂接到这套传动系统上。 */
    if(!Rotor_AttachRotor(context, mainTransmissionIdx, mainRotorIdx)) {
        fprintf(stderr, "Failed to attach main rotor to transmission.\n");
        Rotor_DestroyContext(context);
        return 1;
    }

    /* 构造悬停工况输入，并下发悬停控制量。 */
    hoverStep = BuildBaseStepInput();
    hoverControl = BuildHoverControl();
    Rotor_SetControl(context, mainRotorIdx, &hoverControl);

    /* 先重置一次 transmission，确保内部状态从干净初值开始。 */
    Rotor_ResetTransmission(context, mainTransmissionIdx,
        mainTransmissionConfig.initial_rel_rpm);

    /* 跑一段预热步，让旋翼转速和状态收敛到稳定工况。 */
    for(i = 0; i < 120; ++i) {
        Rotor_StepTransmission(context, mainTransmissionIdx, &hoverStep);
    }

    /* 输出悬停场景结果。 */
    RunScenario(context, mainTransmissionIdx, mainRotorIdx, "hover", &hoverStep);

    /* 基于悬停输入构造前飞场景，并加入一点前向周期变距。 */
    forwardFlightStep = hoverStep;
    forwardFlightStep.state.v[0] = 25.0f;
    forwardFlightControl = BuildHoverControl();
    forwardFlightControl.cyclic_ele = 0.05f;
    Rotor_SetControl(context, mainRotorIdx, &forwardFlightControl);

    /* 输出前飞场景结果。 */
    RunScenario(context, mainTransmissionIdx, mainRotorIdx, "forward_flight",
        &forwardFlightStep);

    /* 程序结束前释放旋翼上下文。 */
    Rotor_DestroyContext(context);
    return 0;
}
