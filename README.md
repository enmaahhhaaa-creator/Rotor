# Rotor 工作区

## 项目目的

本项目的目标是把 YASim 里的旋翼模型从 FlightGear 整体框架中拆出来，整理成一个可单独编译、可单独调用的工程。

当前阶段已经完成这些事情：

- 把旋翼核心源码从原始 YASim 目录拷贝到当前 `Rotor/` 工作区。
- 在 `Rotor/` 下建立 VS2010 解决方案，包含一个示例程序工程和一个静态库工程。
- 对外收口为一个纯 C 头文件 `include/Rotor.h`。
- 把“传动系统”和“旋翼”拆成两个层次：
  - `transmission` 对应原始 YASim 的一套 `rotorgear`
  - `rotor` 对应原始 YASim 的一个 `rotor`
- 支持“先创建 transmission，再创建 rotor，最后把 rotor 挂接到 transmission 上”。
- 用 `RotorDemo/main.c` 演示了“创建 transmission -> 创建 rotor -> 挂接 -> 设置控制 -> 步进计算 -> 读取输出”的完整调用流程。

换句话说，这个目录当前不是整机飞行动力学工程，而是“抽取出来的旋翼求解模块 + 示例调用工程”。

## 原始代码路径

当前工程的原始代码来源目录是：

- `flightgear-2.6.0/src/3_FDM/YASim/`

这次抽取主要参考和拷贝了这些原始文件：

- `flightgear-2.6.0/src/3_FDM/YASim/Rotor.cpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Rotor.hpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Rotorpart.cpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Rotorpart.hpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Math.cpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Math.hpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Glue.cpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Glue.hpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Ground.cpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Ground.hpp`
- `flightgear-2.6.0/src/3_FDM/YASim/Vector.hpp`

参数默认值和 XML 参数含义主要参考：

- `flightgear-2.6.0/src/3_FDM/YASim/FGFDM.cpp`

其中 `<rotor>` 和 `<rotorgear>` 的默认值、字段名、角度单位等，都是从这里对齐过来的。

## 目录结构

- `Rotor.sln`
  VS2010 解决方案，包含两个工程。
- `RotorDemo/`
  控制台示例程序。它以外部调用方的方式调用旋翼库。
- `RotorLib/`
  静态库工程。
- `include/`
  对外公开的 C 头文件目录。
- `src/`
  从 YASim 拷贝出来的旋翼核心源码，以及 C/C++ 封装层实现。

## 公共入口

- `include/Rotor.h`
  唯一的公共头文件。对外暴露纯 C API，以及不透明句柄 `RotorContext`。

当前公共接口的核心调用顺序是：

1. `Rotor_CreateContext()`
2. `Rotor_CreateTransmission(...)` 获取一个 `transmissionIdx`
3. `Rotor_CreateRotor(...)` 获取一个 `rotorIdx`
4. `Rotor_AttachRotor(context, transmissionIdx, rotorIdx)`
5. `Rotor_SetControl(context, rotorIdx, ...)`
6. `Rotor_StepTransmission(context, transmissionIdx, ...)`
7. `Rotor_GetTransmissionOutput(context, transmissionIdx, ...)`
8. `Rotor_GetRotorOutput(context, rotorIdx, ...)`
9. 不再使用时调用 `Rotor_DestroyContext()`

如果要创建多个旋翼实例，就重复调用 `Rotor_CreateRotor(...)`；
如果多个旋翼要共享一套传动系统，就把它们都挂到同一个 `transmissionIdx` 上。

需要注意：

- `Rotor_InitRotorConfig(...)` 和 `Rotor_InitGearConfig(...)` 提供的是“默认初始化模板”，便于你从一组已知起点开始改参数。
- 这些默认值并不等价于一套完整、已经校准好的真实机型参数。
- 真正用于求解时，仍然应该像 `RotorDemo/main.c` 一样，把旋翼几何、气动拟合、功率和控制范围参数明确填完整。

## transmission 和 rotor 的关系

这里必须区分两个概念：

- `transmission`
  负责发动机功率、调速器、刹车、相对转速动态等共享状态
- `rotor`
  负责单个旋翼的几何、气动、挥舞和力/力矩计算

一个 `transmission` 可以挂多个 `rotor`。

这样设计的原因是：

- 如果主旋翼和尾桨共享同一套传动系统，那么它们应该共享同一套 `rotorgear` 状态
- 仅仅给两个独立实例传相同参数，并不等于它们共享同一个 transmission
- 真正的共享传动，关键是共享同一套运行时状态，而不是共享同一套初始参数

## Demo 调用说明

示例程序在：

- `RotorDemo/main.c`

它演示的是“单 transmission + 单 rotor”的完整流程。

`main.c` 里当前使用的命名约定是：

- `mainTransmissionConfig`
  transmission 的配置，类型是 `RotorGearConfig`
- `mainRotorConfig`
  rotor 的配置，类型是 `RotorConfig`
- `hoverStep`
  某一步的外部步进输入，类型是 `RotorStepInput`
- `hoverControl`
  某一步对旋翼的控制输入，类型是 `RotorControlInput`

这些输入的职责不同：

- `RotorStepInput`
  描述“当前这一步外部状态是什么”，例如位置、姿态、速度、风、空气密度、发动机状态
- `RotorControlInput`
  描述“当前这一步旋翼控制量是什么”，例如总距、周期变距、倾转、balance

可以把它理解成：

- `step` 负责描述环境和机体状态
- `control` 负责描述操纵输入

## VS2010 说明

- `RotorDemo` 现在通过链接 `RotorLib` 来使用旋翼能力，不再直接编译旋翼核心源码。
- `RotorLib` 持有从 YASim 拷贝出来的旋翼核心以及封装代码。
- C++ 的 manager/system 头文件属于内部实现，只放在 `src/` 下，不对外公开。
- 对外公共头已经收口为一个 `include/Rotor.h`，适合后续从外部 C 工程直接接入。

## 当前状态

当前工程已经验证过以下链路：

- C++ 核心源码可以独立编译
- 可以生成静态库
- `RotorDemo/main.c` 可以作为 C 调用方链接这个静态库
- 示例中的悬停和前飞工况可以正常输出力、力矩、转速和发动机扭矩结果
- 一个 transmission 下挂接多个 rotor 的共享传动场景可以正常求解，并共享同一套转速和发动机扭矩状态
- reset 已改成真正重建内部状态，而不是只修改单个转速变量

也就是说，“源码抽取 -> transmission/rotor 分层封装 -> 静态库 -> 外部调用” 这一条链已经打通。

## 下一步建议

- 把 `RotorDemo/main.c` 里的示例参数替换成真实机型的 transmission 和 rotor 定义。
- 如果要验证主旋翼加尾桨，可以再创建一个 rotor，并把它挂到同一个 transmission 上。
- 如果后续准备接入外部 C 工程，可以把 `Rotor.h` 的布局进一步固定下来，再开始正式集成。
