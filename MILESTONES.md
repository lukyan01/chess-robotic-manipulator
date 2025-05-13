# Development Milestones

This document outlines the detailed steps for implementing the PID controller and trajectory generator for the 7-DoF manipulator project.

## Phase 1: Project Setup & Environment Configuration (Week 1)

### Milestone 1.1: Environment Setup
- [X] Create project folder structure
- [X] Set up MATLAB path for required directories
- [X] Verify existing dynamics functions (robot_inertia.m, robot_coriolis.m, robot_gravity.m)
- [X] Using ik.m create an inverse kinematics function with propper error handling and validations

### Milestone 1.2: Plant Model Implementation
- [ ] Create Simulink model for the manipulator dynamics
- [ ] Implement state-space representation (q, q̇ as states)
- [ ] Add joint limits using saturation blocks
- [ ] Test open-loop response to constant torques
- [ ] Validate against analytical solutions

## Phase 2: Trajectory Generation

### Milestone 2.1: Basic Trajectory Generation
- [ ] Implement point-to-point quintic polynomial trajectories that output q(t), qd(t), qdd(t)
- [ ] Create joint-space trajectory generator function
- [ ] Create Cartesian-space trajectory generator
- [ ] Add safety checks for kinematic limitations
- [ ] Create visualization tools for trajectories

### Milestone 2.2: Chess-Specific Trajectories
- [ ] Implement chess coordinate conversion function
- [ ] Create chess piece movement trajectories
  - [ ] Lift phase (vertical motion)
  - [ ] Transport phase (horizontal motion)
  - [ ] Place phase (vertical motion)
- [ ] Add collision avoidance considerations
- [ ] Test trajectories with forward kinematics validation

## Phase 3: PID Controller Implementation

### Milestone 3.1: Basic PID Implementation
- [ ] Create Simulink PID controller subsystem
- [ ] Implement individual joint PID controllers
- [ ] Add anti-windup protection
- [ ] Set up tuning interface for PID gains
- [ ] Test response to step inputs

### Milestone 3.2: Feedforward Compensation
- [ ] Add gravity compensation term (G(q))
- [ ] Implement feedforward dynamics compensation (M(q)q̈ᵈ + C(q,q̇)q̇)
- [ ] Create torque saturation and safety limits
- [ ] Test improvement over basic PID

### Milestone 3.3: Integration and Testing
- [ ] Connect trajectory generator to PID controller
- [ ] Set up performance metrics calculation
  - [ ] Position tracking error
  - [ ] Velocity tracking error
  - [ ] Control effort measurement
- [ ] Implement data logging for analysis
- [ ] Test complete system with chess piece movements

## Phase 4: PID Controller Tuning and Optimization (Week 4)

### Milestone 4.1: Systematic Tuning
- [ ] Use Simulink's PID Tuner for initial gains
- [ ] Implement systematic tuning procedure
  - [ ] Ziegler-Nichols method application
  - [ ] Iterative tuning based on performance metrics
- [ ] Document optimal gains for different tasks
- [ ] Create gain scheduling framework (if necessary)

### Milestone 4.2: Robustness Evaluation
- [ ] Add parameter variations (±20% mass and link length)
- [ ] Implement external disturbance inputs
- [ ] Test with different payload masses
- [ ] Evaluate controller robustness metrics
- [ ] Adjust gains to improve robustness

### Milestone 4.3: Performance Optimization
- [ ] Optimize trajectory parameters for smoother motion
- [ ] Fine-tune PID gains for best performance
- [ ] Analyze computational requirements
- [ ] Optimize model for simulation speed
- [ ] Create scripts for automated testing

## Phase 5: Full System Integration (Week 5)

### Milestone 5.1: Complete Simulation Environment
- [ ] Create master Simulink model with switchable controllers
- [ ] Develop comprehensive test suite
- [ ] Implement performance comparison tools
- [ ] Add real-time visualization
- [ ] Optimize simulation performance

### Milestone 5.2: Documentation and Analysis
- [ ] Document controller design and parameters
- [ ] Create performance report template
- [ ] Generate benchmark results
- [ ] Compare with theoretical limitations
- [ ] Prepare for MPC controller implementation

## Phase 6: Model Predictive Control (Future Work)

### Milestone 6.1: System Identification
- [ ] Create linearized models at key operating points
- [ ] Validate linearized models against nonlinear dynamics
- [ ] Determine model uncertainty bounds
- [ ] Set up state estimation (if needed)

### Milestone 6.2: MPC Design
- [ ] Define MPC problem formulation
  - [ ] Prediction and control horizons
  - [ ] Cost function design
  - [ ] Constraints specification
- [ ] Use MPC Designer App to create controller
- [ ] Export MPC controller to Simulink
- [ ] Test basic MPC functionality

### Milestone 6.3: MPC Tuning and Integration
- [ ] Optimize MPC parameters
- [ ] Connect to trajectory generator
- [ ] Implement safety mechanisms
- [ ] Compare performance with PID controller
- [ ] Document findings and recommendations

## Implementation Details

### Setting up the PID Controller

The PID controller should be implemented according to this control law:

```
τ = Kp(qᵈ-q) + Ki∫(qᵈ-q)dt + Kd(q̇ᵈ-q̇) + M(q)q̈ᵈ + C(q,q̇)q̇ + G(q)
```

Step-by-step implementation in Simulink:

1. Create a subsystem with inputs:
   - Current joint positions (q)
   - Current joint velocities (q̇)
   - Desired joint positions (qᵈ)
   - Desired joint velocities (q̇ᵈ)
   - Desired joint accelerations (q̈ᵈ)

2. Calculate tracking errors:
   - Position error: e = qᵈ - q
   - Velocity error: ė = q̇ᵈ - q̇

3. PID computation:
   - Proportional term: Kp × e
   - Integral term: Ki × ∫e dt (with anti-windup)
   - Derivative term: Kd × ė

4. Feedforward computation:
   - Inertia contribution: M(q) × q̈ᵈ
   - Coriolis contribution: C(q,q̇) × q̇
   - Gravity compensation: G(q)

5. Sum all terms to get joint torques (τ)

6. Apply torque limits and safety checks

### Trajectory Generator Implementation

The trajectory generator should provide smooth paths with continuous position, velocity, and acceleration profiles. For chess piece movements:

1. Define waypoints:
   - Starting position (on source square)
   - Lifted position above source
   - Lifted position above destination
   - Final position (on destination square)

2. Generate quintic polynomial trajectories between waypoints:
   - s(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
   - Solve for coefficients using boundary conditions:
     - s(0) = s₀, s(T) = s₁
     - ṡ(0) = v₀, ṡ(T) = v₁
     - s̈(0) = a₀, s̈(T) = a₁

3. Convert from task space to joint space using inverse kinematics

4. Add safety checks to ensure trajectory stays within robot workspace

### Gain Tuning Process

Follow this systematic approach for tuning PID gains:

1. Initial gains:
   - Start with low proportional gains (Kp)
   - Set Ki and Kd to zero

2. Increase Kp until system responds quickly but without excessive overshoot

3. Add derivative gain (Kd) to improve damping

4. Add small integral gain (Ki) to eliminate steady-state error

5. Fine-tune all gains to optimize performance metrics

6. Test with various trajectories and adjust as needed

7. Consider gain scheduling if performance varies significantly across workspace

## Success Criteria

The PID controller and trajectory generator implementation will be considered successful when:

1. The manipulator can accurately follow trajectories with:
   - Position tracking error < 0.5% of workspace size
   - Smooth motion without oscillations
   - Appropriate response to disturbances

2. Chess piece movements are executed with:
   - Appropriate lifting height
   - Smooth transitions between movement phases
   - Accurate final positioning

3. The system is robust to:
   - Parameter variations (±20%)
   - Small external disturbances
   - Different payloads

4. The implementation is well-documented and ready for extension to MPC control