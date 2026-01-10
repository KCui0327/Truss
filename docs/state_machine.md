# Truss Robotic Arm – State Machine Specification

## Overview
This document defines the finite state machine (FSM) governing the robotic arm
control logic for the Truss autonomous strawberry harvesting robot.

The FSM ensures:
- Deterministic control flow
- Safe operation
- Clear separation between perception, planning, and actuation
- Compliance with R3 (Emergency Stop)

The FSM is implemented using the `transitions` Python library.

---

## States

### IDLE
**Description**
- System initialized but inactive
- Awaiting start command

**Entry Actions**
- Initialize internal state
- Reset error flags

---

### HOME
**Description**
- Arm moves to a known safe configuration

**Entry Actions**
- Command arm to home joint angles
- Validate servo feedback

---

### PERCEIVE
**Description**
- Acquire images
- Run ML inference
- Compute 3D strawberry pose

**Exit Conditions**
- Valid ripe fruit detected → PLAN
- No valid target → HOME

---

### PLAN
**Description**
- Compute inverse kinematics
- Generate joint-space trajectory via spline interpolation

**Exit Conditions**
- Valid plan → MOVE
- IK failure → HOME

---

### MOVE
**Description**
- Execute planned joint trajectory

**Entry Actions**
- Send commands to servo bus
- Monitor joint convergence

**Exit Conditions**
- Position reached → CUT
- Timeout or error → HALTED

---

### CUT
**Description**
- Activate scissor end-effector

**Exit Conditions**
- Successful cut → RETURN
- Failure → HALTED

---

### RETURN
**Description**
- Return arm to HOME position

**Exit Conditions**
- Completed → PERCEIVE

---

### HALTED
**Description**
- Emergency stop state
- All motion disabled

**Entry Actions**
- Immediately disable servo outputs
- Log halt event

**Exit Conditions**
- Manual reset only

---

## State Transitions

| From | To | Trigger |
|----|----|----|
| IDLE | HOME | start |
| HOME | PERCEIVE | homed |
| PERCEIVE | PLAN | target_detected |
| PLAN | MOVE | plan_valid |
| MOVE | CUT | target_reached |
| CUT | RETURN | cut_success |
| RETURN | PERCEIVE | ready |
| * | HALTED | emergency_stop |

---

## Safety Guarantees
- Emergency stop preempts all states
- No hardware actuation occurs outside MOVE/CUT
- All failures converge to safe states

---

## Implementation Notes
- FSM implemented in `src/components/arm_fsm.py`
- Hardware access abstracted via `servo_bus`
- Simulation uses `SimServoBus`
- Raspberry Pi uses real servo drivers

---

## Compliance
- R1: 3–5 s harvest cycle
- R3: ≤ 1.25 s emergency halt
- R4: ≤ 1 cm positioning accuracy
