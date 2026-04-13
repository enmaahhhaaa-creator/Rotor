# Rotor Workspace

## Structure

- `Rotor.sln`
  VS2010 solution with two projects.
- `RotorDemo/`
  Console host program. It calls the library as an external user.
- `RotorLib/`
  Static library project.
- `include/`
  Public C header.
- `src/`
  Rotor core sources copied from YASim plus the C/C++ wrapper layer.

## Public Entry Points

- `include/Rotor.h`
  The only public header. It exposes a pure C API and an opaque `RotorContext`.

## Current Flow

1. `RotorDemo/main.cpp` builds one `RotorCreateInput` in code.
2. `RotorDemo` calls `Rotor_CreateContext()`.
3. `RotorDemo` calls `Rotor_CreateRotor(...)` and receives an `idx`.
4. `RotorDemo` calls `Rotor_SetControl(idx, ...)`.
5. `RotorDemo` calls `Rotor_Step(idx, ...)`.
6. `RotorDemo` calls `Rotor_GetOutput(idx, ...)`.
7. If you need more rotors later, create more instances and keep each one separate by `idx`.

## VS2010 Notes

- `RotorDemo` now links `RotorLib`; it no longer compiles the rotor core sources directly.
- `RotorLib` owns the copied YASim rotor core and the wrapper code.
- The C++ manager/system headers are internal and live under `src/`.

## Next Suggested Work

- Replace the demo config in `RotorDemo/main.cpp` with a real aircraft rotor definition.
- Add a second rotor by calling `Rotor_CreateRotor(...)` again if you want to validate main rotor + tail rotor.
- If needed, freeze the layout of `Rotor.h` and start integrating it from your external C codebase.
