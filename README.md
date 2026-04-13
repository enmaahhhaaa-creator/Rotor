# Rotor 工作区

## 项目目的

本项目的目标是把 YASim 里的旋翼模型从 FlightGear 整体框架中拆出来，整理成一个可单独编译、可单独调用的工程。

当前阶段已经完成这些事情：

- 把旋翼核心源码从原始 YASim 目录拷贝到当前 `Rotor/` 工作区。
- 在 `Rotor/` 下建立 VS2010 解决方案，包含一个示例程序工程和一个静态库工程。
- 对外收口为一个纯 C 头文件 `include/Rotor.h`。
- 通过 `idx` 管理多个独立旋翼实例；你可以只创建一个旋翼，也可以创建多个旋翼实例分别计算。
- 用 `RotorDemo/main.c` 演示了“创建旋翼 -> 设置控制 -> 步进计算 -> 读取输出”的完整调用流程。

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

公共接口的核心调用顺序是：

1. `Rotor_CreateContext()`
2. `Rotor_CreateRotor(...)` 获取一个 `idx`
3. `Rotor_SetControl(context, idx, ...)`
4. `Rotor_Step(context, idx, ...)`
5. `Rotor_GetOutput(context, idx, ...)`
6. 不再使用时调用 `Rotor_DestroyContext()`

如果要创建多个旋翼实例，就重复调用 `Rotor_CreateRotor(...)`，然后通过不同的 `idx` 分开管理。

## Demo 调用说明

示例程序在：

- `RotorDemo/main.c`

它演示的是“单旋翼实例”的完整流程。

`main.c` 里现在使用的命名约定是：

- `hoverStep`
  表示某一步的外部步进输入，类型是 `RotorStepInput`
- `hoverControl`
  表示这一工况下给旋翼的控制输入，类型是 `RotorControlInput`
- `forwardFlightStep`
  表示前飞工况下的步进输入
- `forwardFlightControl`
  表示前飞工况下的控制输入

这两个输入的职责不同：

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
- 示例中的悬停和前飞工况可以正常输出力、力矩和转速结果

也就是说，“源码抽取 -> C 封装 -> 静态库 -> 外部调用” 这一条链已经打通。

## 下一步建议

- 把 `RotorDemo/main.c` 里的示例参数替换成真实机型的旋翼定义。
- 如果要验证主旋翼加尾桨，可以再次调用 `Rotor_CreateRotor(...)` 创建第二个实例。
- 如果后续准备接入外部 C 工程，可以把 `Rotor.h` 的布局进一步固定下来，再开始正式集成。
