#include <stdio.h>
#include <string.h>

#include "../include/Rotor.h"

static RotorCreateInput BuildMainRotorCreateInput(void)
{
    RotorCreateInput createInput;
    RotorConfig* rotorConfig;
    RotorGearConfig* gearConfig;

    /* 先用库内默认值填充整套创建参数。 */
    Rotor_InitCreateInput(&createInput);

    /* 取出便于填写的快捷指针。 */
    rotorConfig = &createInput.rotor;
    gearConfig = &createInput.gear;

    /* 配置旋翼名称，便于后续调试和日志识别。 */
    strncpy(rotorConfig->name, "main", sizeof(rotorConfig->name) - 1);
    rotorConfig->name[sizeof(rotorConfig->name) - 1] = '\0';

    /* 设置桨毂位置和旋翼坐标轴方向。 */
    rotorConfig->base[0] = 0.0f;
    rotorConfig->base[1] = 0.0f;
    rotorConfig->base[2] = 1.5f;
    rotorConfig->normal[0] = 0.0f;
    rotorConfig->normal[1] = 0.0f;
    rotorConfig->normal[2] = 1.0f;
    rotorConfig->forward[0] = 1.0f;
    rotorConfig->forward[1] = 0.0f;
    rotorConfig->forward[2] = 0.0f;

    /* 设置主旋翼的基础几何和动力学参数。 */
    rotorConfig->diameter_m = 10.2f;
    rotorConfig->weight_per_blade_lb = 44.0f;
    rotorConfig->num_blades = 4.0f;
    rotorConfig->rel_blade_center = 0.7f;
    rotorConfig->dynamic = 0.7f;
    rotorConfig->delta3 = 0.0f;
    rotorConfig->delta = 1.0f;
    rotorConfig->rpm = 424.0f;

    /* 设置总距和周期变距的控制范围。 */
    rotorConfig->max_collective_deg = 15.8f;
    rotorConfig->min_collective_deg = -0.2f;
    rotorConfig->max_cyclic_ail_deg = 7.6f;
    rotorConfig->max_cyclic_ele_deg = 4.94f;
    rotorConfig->min_cyclic_ail_deg = -7.6f;
    rotorConfig->min_cyclic_ele_deg = -4.94f;

    /* 设置用于气动拟合的参考点参数。 */
    rotorConfig->pitch_a_deg = 10.0f;
    rotorConfig->pitch_b_deg = 10.0f;
    rotorConfig->force_at_pitch_a_lb = 3000.0f;
    rotorConfig->power_at_pitch_0_kw = 300.0f;
    rotorConfig->power_at_pitch_b_kw = 3000.0f;

    /* 直接给出空气动力系数和离散化设置。 */
    rotorConfig->airfoil_lift_coefficient = 6.0f;
    rotorConfig->airfoil_drag_coefficient0 = 0.012f;
    rotorConfig->airfoil_drag_coefficient1 = 0.08f;
    rotorConfig->number_of_parts = 16.0f;
    rotorConfig->rotor_correction_factor = 0.65f;

    /* 设置发动机和传动系统相关参数。 */
    gearConfig->max_power_engine_kw = 650.0f;
    gearConfig->engine_prop_factor = 0.05f;
    gearConfig->max_power_rotor_brake_kw = 20.0f;
    gearConfig->rotorgear_friction_kw = 1.0f;
    gearConfig->engine_accel_limit = 0.05f;

    return createInput;
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

static void RunScenario(RotorContext* context, int idx, const char* name,
    const RotorStepInput* stepInput)
{
    RotorOutput rotorOutput;

    /* 每次场景求解前都把输出结构清零。 */
    Rotor_InitOutput(&rotorOutput);

    /* 打印当前场景名，便于区分输出。 */
    printf("Scenario: %s\n", name);

    /* 先执行一步旋翼求解。 */
    if(!Rotor_Step(context, idx, stepInput)) {
        printf("  step failed\n");
        return;
    }

    /* 再读取本次求解生成的输出结果。 */
    if(!Rotor_GetOutput(context, idx, &rotorOutput)) {
        printf("  step failed\n");
        return;
    }

    /* 逐项打印力、力矩和转速结果。 */
    PrintVec3("  total_force_local", rotorOutput.total_force);
    PrintVec3("  total_moment_local", rotorOutput.total_moment);
    PrintVec3("  gear_torque_local", rotorOutput.gear_torque);
    PrintVec3("  rotor_force_local", rotorOutput.rotor.force);
    PrintVec3("  rotor_moment_local", rotorOutput.rotor.moment);
    printf("  rotor_scalar_torque: %.3f\n", rotorOutput.rotor.scalar_torque);
    printf("  rotor_rpm: %.3f\n", rotorOutput.rotor.rpm);
    printf("  rotor_omega_rel(current/next): %.3f / %.3f\n",
        rotorOutput.rotor.omega_rel, rotorOutput.rotor.omega_rel_next);
    printf("\n");
}

int main(void)
{
    RotorContext* context;
    RotorCreateInput createInput;
    int mainRotorIdx;
    RotorStepInput hoverStep;
    RotorControlInput hoverControl;
    RotorStepInput forwardFlightStep;
    RotorControlInput forwardFlightControl;
    int i;

    /* 创建旋翼上下文，并准备单旋翼创建参数。 */
    context = Rotor_CreateContext();
    createInput = BuildMainRotorCreateInput();

    /* 在上下文中创建一个主旋翼实例，并拿到对应的 idx。 */
    mainRotorIdx = Rotor_CreateRotor(context, &createInput);
    if(mainRotorIdx < 0) {
        /* 创建失败时直接释放上下文并退出。 */
        fprintf(stderr, "Failed to create main rotor model.\n");
        Rotor_DestroyContext(context);
        return 1;
    }

    /* 构造悬停工况输入，并下发悬停控制量。 */
    hoverStep = BuildBaseStepInput();
    hoverControl = BuildHoverControl();
    Rotor_SetControl(context, mainRotorIdx, &hoverControl);

    /* 先跑一段预热步，让旋翼转速和状态收敛到稳定工况。 */
    for(i = 0; i < 120; ++i) {
        Rotor_Step(context, mainRotorIdx, &hoverStep);
    }

    /* 输出悬停场景结果。 */
    RunScenario(context, mainRotorIdx, "hover", &hoverStep);

    /* 基于悬停输入构造前飞场景，并加入一点前向周期变距。 */
    forwardFlightStep = hoverStep;
    forwardFlightStep.state.v[0] = 25.0f;
    forwardFlightControl = BuildHoverControl();
    forwardFlightControl.cyclic_ele = 0.05f;
    Rotor_SetControl(context, mainRotorIdx, &forwardFlightControl);

    /* 输出前飞场景结果。 */
    RunScenario(context, mainRotorIdx, "forward_flight", &forwardFlightStep);

    /* 程序结束前释放旋翼上下文。 */
    Rotor_DestroyContext(context);
    return 0;
}
