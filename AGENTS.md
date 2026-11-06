# AGENTS

## Repo Intent

This repository is the launch-product monorepo for `RVClaw EdgeOne DevKit`.
This delivery targets `K1 MUSE Pi Pro + V550 ROS2`.

## Ownership by Product Domain

- `robot/ros2_ws/src/rvclaw_base/`: `M2` base control
- `robot/ros2_ws/src/rvclaw_perception/`: `M3` perception integration
- `robot/ros2_ws/src/rvclaw_algo/`: `M4` inspection algorithms
- `services/rvclaw_api/`: API layer consumed by Studio
- `services/rvclaw_report/`: report generation
- `studio/web-vue/`: `M5` Studio frontend source; `studio/web/` is generated
- `contracts/`: shared schemas and interface contracts

## Working Rules

1. Keep changes scoped to one product domain unless the task is explicitly cross-module.
2. Update `contracts/` before changing producer and consumer payloads.
3. Do not reintroduce the old `Demo Claw` directory layout on `main`.
4. Preserve launch-product boundaries: low-speed inspection, point trigger, local inference, local Studio.
