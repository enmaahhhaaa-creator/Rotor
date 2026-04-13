#ifndef ROTOR_H
#define ROTOR_H

/*
 * 抽取后的 YASim 旋翼模型公共 C 接口。
 *
 * 本头文件使用的坐标约定：
 * - “local/body”表示机体坐标系，旋翼几何输入和力/力矩输出都在这个坐标系下。
 * - “global”表示世界坐标系，state.pos/state.v/state.rot/wind_global 使用这个坐标系。
 * - state.orient 与原始 YASim 的 State::orient 含义一致，必须能把 global 向量变换到
 *   local/body。
 *
 * 下文提到的“初始化默认值”来自以下初始化函数：
 * - Rotor_InitCreateInput
 * - Rotor_InitControlInput
 * - Rotor_InitStepInput
 *
 * 如果某个默认值来自原始 YASim 的 XML 解析默认值，会标注为“YASim XML 默认值”；
 * 否则表示它来自保留下来的 YASim 内部旋翼默认值。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RotorContext RotorContext;

/*
 * 可选的地面探测回调。
 * - pos：采样点，global 坐标系，单位 m。
 * - plane：写回地面平面参数，满足 n.x * x + n.y * y + n.z * z = d，global 坐标系。
 * - vel：写回该点地面速度，global 坐标系，单位 m/s。
 */
typedef void (*RotorGroundPlaneFn)(void* user_data, const double pos[3],
    double plane[4], float vel[3]);

typedef struct {
    /* 地面查询回调。设为 0 表示不查询地面，地面效应逻辑会关闭。 */
    RotorGroundPlaneFn get_ground_plane;
    /* 透传给 get_ground_plane 的用户指针。 */
    void* user_data;
} RotorGroundProvider;

typedef struct {
    /* 飞行器参考位置。坐标系：global。单位：m。初始化默认值：{0,0,0}。 */
    double pos[3];
    /* global->local/body 的 3x3 方向矩阵。坐标系：global -> local。单位：无。初始化默认值：单位阵。 */
    float orient[9];
    /* 飞行器线速度。坐标系：global。单位：m/s。初始化默认值：{0,0,0}。 */
    float v[3];
    /* 飞行器角速度。坐标系：global。单位：rad/s。初始化默认值：{0,0,0}。 */
    float rot[3];
} RotorState;

typedef struct {
    /* 归一化总距输入。单位：[-1,1]。初始化默认值：0.0。-1 对应 min_collective_deg，+1 对应 max_collective_deg。 */
    float collective;
    /* 归一化横向周期变距输入。单位：[-1,1]。初始化默认值：0.0。方向约定沿用原始 YASim。 */
    float cyclic_ail;
    /* 归一化纵向周期变距输入。单位：[-1,1]。初始化默认值：0.0。方向约定沿用原始 YASim。 */
    float cyclic_ele;
    /* 归一化滚转倾转输入。单位：[-1,1]。初始化默认值：0.0。映射到 min_tilt_roll_deg..max_tilt_roll_deg。 */
    float tilt_roll;
    /* 归一化俯仰倾转输入。单位：[-1,1]。初始化默认值：0.0。映射到 min_tilt_pitch_deg..max_tilt_pitch_deg。 */
    float tilt_pitch;
    /* 归一化偏航倾转输入。单位：[-1,1]。初始化默认值：0.0。映射到 min_tilt_yaw_deg..max_tilt_yaw_deg。 */
    float tilt_yaw;
    /* 外部旋翼平衡/健康度因子。单位：[-1,1]。初始化默认值：1.0。推荐值：健康旋翼保持 1.0。 */
    float balance;
} RotorControlInput;

typedef struct {
    /* 旋翼桨毂基点位置。坐标系：local/body。单位：m。初始化默认值：{0,0,0}。原始 XML 一般会显式给 x/y/z。 */
    float base[3];
    /* 旋翼盘面法向。坐标系：local/body。单位：单位向量。初始化默认值：{0,0,1}。原始 XML 一般会显式给 nx/ny/nz。 */
    float normal[3];
    /* 旋翼前向参考方向，用于定义方位角/相位。坐标系：local/body。单位：单位向量。初始化默认值：{1,0,0}。原始 XML 一般会显式给 fx/fy/fz。 */
    float forward[3];

    /* 周期变距横向上限。单位：deg。初始化默认值：7.6（YASim XML 默认值）。 */
    float max_cyclic_ail_deg;
    /* 周期变距纵向上限。单位：deg。初始化默认值：4.94（YASim XML 默认值）。 */
    float max_cyclic_ele_deg;
    /* 周期变距横向下限。单位：deg。初始化默认值：-7.6（YASim XML 默认值）。 */
    float min_cyclic_ail_deg;
    /* 周期变距纵向下限。单位：deg。初始化默认值：-4.94（YASim XML 默认值）。 */
    float min_cyclic_ele_deg;
    /* 总距上限。单位：deg。初始化默认值：15.8（YASim XML 默认值）。 */
    float max_collective_deg;
    /* 总距下限。单位：deg。初始化默认值：-0.2（YASim XML 默认值）。 */
    float min_collective_deg;
    /* 旋翼直径。单位：m。初始化默认值：10.2（YASim XML 默认值）。 */
    float diameter_m;
    /* 单片桨叶重量，用于惯量和挥舞动力学。单位：lb。初始化默认值：44.0（YASim XML 默认值）。 */
    float weight_per_blade_lb;
    /* 桨叶数。为了兼容旧实现，类型保留为 float。单位：片数。初始化默认值：4.0（YASim XML 默认值）。 */
    float num_blades;
    /* 桨叶受力等效中心的相对半径位置。单位：半径比例。初始化默认值：0.7（YASim XML 默认值）。 */
    float rel_blade_center;
    /* 原始 YASim 挥舞/响应动态系数。单位：无。初始化默认值：0.7（YASim XML 默认值）。 */
    float dynamic;
    /* 原始 YASim 的 delta-3 耦合系数。单位：无。初始化默认值：0.0（YASim XML 默认值）。 */
    float delta3;
    /* 原始 YASim 的附加动力学系数 delta。单位：无。初始化默认值：0.0（YASim XML 默认值）。 */
    float delta;
    /* 旧版 rotorpart 计算里使用的 translift 系数。单位：无。初始化默认值：0.05（YASim XML 默认值）。 */
    float translift;
    /* 旧版阻力因子，对应原始 YASim 的 C2 参数。单位：无。初始化默认值：1.0（YASim XML 默认值）。 */
    float drag_factor;
    /* 旋翼内部积分/更新频率。单位：Hz。初始化默认值：120.0（YASim XML 默认值）。 */
    float steps_per_second;
    /* 初始旋翼相位偏置。单位：rad。初始化默认值：0.0，对应原始 XML 中 phi0=0 deg。 */
    float phi0_rad;
    /* 标称旋翼转速。单位：rpm。初始化默认值：424.0（YASim XML 默认值）。 */
    float rpm;
    /* 挥舞铰相对半径位置。单位：半径比例。初始化默认值：0.07（YASim XML 默认值）。 */
    float rel_len_flap_hinge;
    /* 中性挥舞角。单位：rad。初始化默认值：-0.0872665，对应原始 XML 中 flap0=-5 deg。 */
    float flap0_rad;
    /* 挥舞角下限。单位：rad。初始化默认值：-0.2617994，对应原始 XML 中 flapmin=-15 deg。 */
    float flapmin_rad;
    /* 挥舞角上限。单位：rad。初始化默认值：0.2617994，对应原始 XML 中 flapmax=15 deg。 */
    float flapmax_rad;
    /* flap0 缩放因子。单位：无。初始化默认值：1.0（YASim XML 默认值）。 */
    float flap0_factor;
    /* 摇摆铰阻尼系数。单位：无。初始化默认值：0.0001（YASim XML 默认值）。 */
    float teeter_damp;
    /* 摇摆铰阻尼上限。单位：无。初始化默认值：1000.0（YASim XML 默认值）。 */
    float max_teeter_damp;
    /* 摇摆铰相对半径位置。单位：半径比例。初始化默认值：0.01（YASim XML 默认值）。 */
    float rel_len_teeter_hinge;
    /* 静态/内部平衡系数。单位：无。初始化默认值：1.0（YASim XML 默认值）。推荐值：通常保持 1.0，除非你要模拟失衡或损伤。 */
    float balance;
    /* 倾转偏航角下限，对应 tilt_yaw=-1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float min_tilt_yaw_deg;
    /* 倾转俯仰角下限，对应 tilt_pitch=-1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float min_tilt_pitch_deg;
    /* 倾转滚转角下限，对应 tilt_roll=-1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float min_tilt_roll_deg;
    /* 倾转偏航角上限，对应 tilt_yaw=+1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float max_tilt_yaw_deg;
    /* 倾转俯仰角上限，对应 tilt_pitch=+1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float max_tilt_pitch_deg;
    /* 倾转滚转角上限，对应 tilt_roll=+1。单位：deg。初始化默认值：0.0（YASim XML 默认值）。 */
    float max_tilt_roll_deg;
    /* 倾转中心位置。坐标系：local/body。单位：m。初始化默认值：{0,0,0}（YASim XML 默认值）。 */
    float tilt_center[3];
    /* 下洗强度缩放因子。单位：无。初始化默认值：1.0（YASim XML 默认值）。 */
    float downwash_factor;

    /* 旋翼转向标志，沿用原始 YASim 约定。单位：bool(int)。初始化默认值：0。若原始 XML 使用 ccw，则设为 1。 */
    int ccw;
    /* 是否启用 shared flap hinge 逻辑。单位：bool(int)。初始化默认值：0。只有原始 XML 使用 sharedflaphinge 时才设为 1。 */
    int shared_flap_hinge;
    /* 是否关闭反扭矩贡献。单位：bool(int)。初始化默认值：0。只有原始 XML 使用 notorque 时才设为 1。 */
    int no_torque;

    /* 可选旋翼名称，仅用于调试/诊断。单位：无。初始化默认值：空字符串。 */
    char name[64];

    /* 参考桨距点 A，用于在未显式给空气动力系数时拟合系数。单位：deg。初始化默认值：10.0（YASim XML 默认值）。 */
    float pitch_a_deg;
    /* 参考桨距点 B，用于在未显式给空气动力系数时拟合系数。单位：deg。初始化默认值：10.0（YASim XML 默认值）。 */
    float pitch_b_deg;
    /* 点 A 桨距对应的升力，用于系数拟合。单位：lb-force。初始化默认值：3000.0（YASim XML 默认值）。 */
    float force_at_pitch_a_lb;
    /* 0 度桨距对应的功率，用于系数拟合。单位：kW。初始化默认值：300.0（YASim XML 默认值）。 */
    float power_at_pitch_0_kw;
    /* 点 B 桨距对应的功率，用于系数拟合。单位：kW。初始化默认值：3000.0（YASim XML 默认值）。 */
    float power_at_pitch_b_kw;

    /* 平移升力起效速度尺度。单位：m/s。初始化默认值：20.0（YASim 内部默认值）。 */
    float translift_ve;
    /* 平移升力最大缩放倍数。单位：无。初始化默认值：1.3（YASim 内部默认值）。 */
    float translift_maxfactor;
    /* 地面效应系数，模型形式约为 1 + diameter / altitude * constant。单位：无。初始化默认值：0.1（YASim 内部默认值）。 */
    float ground_effect_constant;
    /* 涡环态升力因子。单位：无。初始化默认值：0.4（YASim 内部默认值）。 */
    float vortex_state_lift_factor;
    /* 涡环态经验参数 C1。单位：无。初始化默认值：0.1（YASim 内部默认值）。 */
    float vortex_state_c1;
    /* 涡环态经验参数 C2。单位：无。初始化默认值：0.0（YASim 内部默认值）。 */
    float vortex_state_c2;
    /* 涡环态经验参数 C3。单位：无。初始化默认值：0.0（YASim 内部默认值）。 */
    float vortex_state_c3;
    /* 涡环态经验指数 E1。单位：无。初始化默认值：1.0（YASim 内部默认值）。 */
    float vortex_state_e1;
    /* 涡环态经验指数 E2。单位：无。初始化默认值：1.0（YASim 内部默认值）。 */
    float vortex_state_e2;
    /* 桨叶扭转角，从根部到翼尖的附加扭转。单位：deg。初始化默认值：0.0（YASim 内部默认值）。 */
    float twist_deg;
    /* 每个 rotorpart 的展向积分段数。单位：段数，类型保留为 float。初始化默认值：1.0（YASim 内部默认值）。 */
    float number_of_segments;
    /* 方位向离散的 rotorpart 数量。单位：个数，类型保留为 float。初始化默认值：4.0（YASim 内部默认值）。推荐值：保持 4 的整数倍。 */
    float number_of_parts;
    /* 计算桨叶迎角时使用的相对半径位置。单位：半径比例。初始化默认值：0.7（YASim 内部默认值）。 */
    float rel_len_where_incidence_is_measured;
    /* 桨叶弦长。单位：m。初始化默认值：0.3（YASim 内部默认值）。 */
    float chord;
    /* 翼尖/翼根弦长比。单位：无。初始化默认值：1.0（YASim 内部默认值）。 */
    float taper;
    /* 零升力迎角。单位：deg。初始化默认值：0.0（YASim 内部默认值）。 */
    float airfoil_incidence_no_lift_deg;
    /* 桨叶起始相对半径位置。单位：半径比例。初始化默认值：0.0（YASim 内部默认值）。 */
    float rel_len_blade_start;
    /* 零前飞速度下的失速迎角。单位：deg。初始化默认值：18.0（YASim 内部默认值）。 */
    float incidence_stall_zero_speed_deg;
    /* 半音速条件下的失速迎角。单位：deg。初始化默认值：14.0（YASim 内部默认值）。 */
    float incidence_stall_half_sonic_speed_deg;
    /* 失速区剩余升力比例。单位：无。初始化默认值：0.28（YASim 内部默认值）。 */
    float lift_factor_stall;
    /* 失速过渡带宽。单位：deg。初始化默认值：2.0（YASim 内部默认值）。 */
    float stall_change_over_deg;
    /* 失速时阻力放大倍数。单位：无。初始化默认值：8.0（YASim 内部默认值）。 */
    float drag_factor_stall;
    /* 显式空气动力升力系数。单位：无。初始化默认值：0.0。推荐值：若希望沿用 YASim 的自动拟合，保持 0.0。 */
    float airfoil_lift_coefficient;
    /* 显式空气动力阻力常数项。单位：无。初始化默认值：0.0。推荐值：若希望沿用 YASim 的自动拟合，保持 0.0。 */
    float airfoil_drag_coefficient0;
    /* 显式空气动力阻力迎角项。单位：无。初始化默认值：0.0。推荐值：若希望沿用 YASim 的自动拟合，保持 0.0。 */
    float airfoil_drag_coefficient1;
    /* 周期变距响应缩放系数。单位：无。初始化默认值：1.0（YASim 内部默认值/高级 XML 参数默认值）。 */
    float cyclic_factor;
    /* 原始 YASim 保留的经验修正系数。单位：无。初始化默认值：0.65（YASim 内部默认值）。 */
    float rotor_correction_factor;
} RotorConfig;

typedef struct {
    /* 旋翼传动模型可用的发动机最大功率。单位：kW。初始化默认值：450.0。与保留的 YASim rotorgear 默认值一致。 */
    float max_power_engine_kw;
    /* 相对转速调速器的比例作用范围。单位：相对转速比。初始化默认值：0.05（YASim rotorgear/XML 默认值）。 */
    float engine_prop_factor;
    /* 原始 YASim gear drag 缩放参数，为兼容性保留。单位：无。初始化默认值：1.0（YASim rotorgear/XML 默认值）。 */
    float yasim_drag_factor;
    /* 原始 YASim gear lift 缩放参数，为兼容性保留。单位：无。初始化默认值：1.0（YASim rotorgear/XML 默认值）。 */
    float yasim_lift_factor;
    /* 最大旋翼刹车功率。单位：kW。初始化默认值：1.0（YASim rotorgear/XML 默认值）。 */
    float max_power_rotor_brake_kw;
    /* 传动系统/刹车常值摩擦功率项。单位：kW。初始化默认值：1.0（YASim rotorgear/XML 默认值）。 */
    float rotorgear_friction_kw;
    /* 发动机驱动的相对转速最大加速度限制。单位：1/s。初始化默认值：0.05（YASim rotorgear/XML 默认值）。 */
    float engine_accel_limit;
    /* 创建或 Reset 时使用的初始相对转速。单位：相对标称转速比。初始化默认值：1.0。推荐值：已带转状态保持 1.0。 */
    float initial_rel_rpm;
} RotorGearConfig;

typedef struct {
    /* 单个旋翼的几何、气动和动力学参数。初始化默认值：Rotor_InitCreateInput 会填充上面的 RotorConfig 默认值。 */
    RotorConfig rotor;
    /* 单个旋翼的传动/发动机参数。初始化默认值：Rotor_InitCreateInput 会填充上面的 RotorGearConfig 默认值。 */
    RotorGearConfig gear;
} RotorCreateInput;

typedef struct {
    /* 当前步的飞行器运动学状态。初始化默认值：零位置/零速度/零角速度，方向矩阵为单位阵。 */
    RotorState state;
    /* 求解步长。单位：s。初始化默认值：1/60。推荐值：与外部仿真主循环步长一致。 */
    float dt;
    /* 空气密度。单位：kg/m^3。初始化默认值：1.184。推荐值：若外部没有大气模型，可先用海平面近似值。 */
    float rho;
    /* 环境风速度。坐标系：global。单位：m/s。初始化默认值：{0,0,0}。 */
    float wind_global[3];
    /* 用于累计力矩的机体重心位置。坐标系：local/body。单位：m。初始化默认值：{0,0,0}。 */
    float cg_local[3];
    /* 发动机是否接通。单位：bool(int)。初始化默认值：1。设为 0 可模拟断发/自转工况。 */
    int engine_on;
    /* 旋翼刹车输入。单位：[0,1]。初始化默认值：0.0。0 表示无刹车，1 表示最大刹车功率。 */
    float rotor_brake;
    /* 送入旋翼传动系统的可用发动机扭矩比例。单位：比例。初始化默认值：1.0。推荐范围：0..1。 */
    float max_rel_torque;
    /* 调速器目标相对转速。单位：相对标称转速比。初始化默认值：1.0。推荐值：定转速工作时保持 1.0。 */
    float rel_target;
    /* 可选地面回调提供器。初始化默认值：{0,0}。为空时地面效应等价于关闭。 */
    RotorGroundProvider ground_provider;
} RotorStepInput;

typedef struct {
    /* 旋翼产生的气动力。坐标系：local/body。单位：N。初始化默认值：{0,0,0}。 */
    float force[3];
    /* 旋翼对 cg_local 的总力矩。坐标系：local/body。单位：N*m。初始化默认值：{0,0,0}。 */
    float moment[3];
    /* 仅由力臂 r x F 产生的力矩分量。坐标系：local/body。单位：N*m。初始化默认值：{0,0,0}。 */
    float arm_moment[3];
    /* 不通过力臂、直接叠加的轴扭矩/挥舞/加速力矩分量。坐标系：local/body。单位：N*m。初始化默认值：{0,0,0}。 */
    float direct_torque[3];
    /* 标量形式的旋翼轴扭矩。单位：N*m。初始化默认值：0.0。 */
    float scalar_torque;
    /* 当前步求解后的相对转速。单位：相对标称转速比。初始化默认值：0.0。 */
    float omega_rel;
    /* 原始 YASim 内部保留的下一步相对转速状态。单位：相对标称转速比。初始化默认值：0.0。 */
    float omega_rel_next;
    /* 实际旋翼转速。单位：rpm。初始化默认值：0.0。 */
    float rpm;
} RotorResult;

typedef struct {
    /* 该旋翼实例输出的总力。坐标系：local/body。单位：N。初始化默认值：{0,0,0}。 */
    float total_force[3];
    /* 该旋翼实例输出的总力矩，已经包含 gear_torque。坐标系：local/body。单位：N*m。初始化默认值：{0,0,0}。 */
    float total_moment[3];
    /* 由传动/刹车/转速变化引入的附加力矩。坐标系：local/body。单位：N*m。初始化默认值：{0,0,0}。 */
    float gear_torque[3];
    /* 等效到发动机轴上的总扭矩。单位：N*m。初始化默认值：0.0。 */
    float engine_torque;
    /* 当前 idx 对应旋翼的详细输出。初始化默认值：全 0 的 RotorResult。 */
    RotorResult rotor;
} RotorOutput;

RotorContext* Rotor_CreateContext(void);
void Rotor_DestroyContext(RotorContext* context);

/* 用上文记录的默认值填充 create_input。 */
void Rotor_InitCreateInput(RotorCreateInput* out_input);
/* 填充中性操纵输入，balance=1.0。 */
void Rotor_InitControlInput(RotorControlInput* out_input);
/* 填充零状态、单位阵方向、dt=1/60、rho=1.184、engine_on=1。 */
void Rotor_InitStepInput(RotorStepInput* out_input);
/* 将所有输出字段清零。 */
void Rotor_InitOutput(RotorOutput* out_output);

int Rotor_CreateRotor(RotorContext* context, const RotorCreateInput* create_input);
int Rotor_GetCount(const RotorContext* context);
int Rotor_SetControl(RotorContext* context, int idx, const RotorControlInput* control);
int Rotor_Reset(RotorContext* context, int idx, float initial_rel_rpm);
int Rotor_Step(RotorContext* context, int idx, const RotorStepInput* input);
int Rotor_GetOutput(const RotorContext* context, int idx, RotorOutput* output);
void Rotor_Clear(RotorContext* context);

#ifdef __cplusplus
}
#endif

#endif
