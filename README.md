# 7-DoF Manipulator Control System

This project implements a comprehensive control system for a 7-DoF robot manipulator with a focus on trajectory generation and multiple control strategies. The system leverages MATLAB/Simulink to design, test, and compare different controllers for precise manipulation tasks.

## Project Overview

The manipulator features a complex kinematic chain with 7 degrees of freedom:
- 2 revolute joints (θ₁, θ₇)
- 2 prismatic joints (d₂, d₃)
- 3 additional revolute joints (θ₄, θ₅, θ₆)

This design enables the manipulator to perform complex tasks like chess piece movements with high precision.

## Repository Structure

```
project_root/
├── models/                     # Simulink models
│   ├── manipulator_pid.slx     # PID controller implementation
│   ├── manipulator_mpc.slx     # MPC controller implementation 
│   └── comparison.slx          # Model for comparing controllers
├── functions/
│   ├── dynamics/               # Dynamics model functions
│   │   ├── robot_inertia.m     # M(q) matrix
│   │   ├── robot_coriolis.m    # C(q,q̇) matrix
│   │   └── robot_gravity.m     # G(q) vector
│   ├── kinematics/             # Kinematics functions
│   │   ├── inverseKinematics.m # IK solver
│   │   ├── getSquareCoord.m    # Chess square to robot coordinate conversion
│   │   └── fk_validation.m     # Forward kinematics for validation
│   └── trajectories/           # Trajectory generation
│       ├── generateLinearTraj.m
│       └── generateChessTraj.m # Chess-specific trajectories
├── scripts/
│   ├── setup.m                 # Initialize workspace variables
│   ├── run_simulations.m       # Run test suite
│   └── analyze_results.m       # Performance analysis
├── controllers/
│   ├── pid_controller.m        # PID with feedforward compensation
│   └── mpc_design.m            # MPC controller design
├── data/
│   └── simulation_results/     # Simulation outputs
├── README.md                   # Project overview (this file)
└── MILESTONES.md               # Development milestones
```

## Features

- **Full Dynamic Model**: Includes inertia matrix (M), Coriolis/centrifugal terms (C), and gravity vector (G)
- **Inverse Kinematics**: Closed-form solution for the 7-DoF chain
- **Trajectory Generation**: Smooth trajectories for point-to-point and chess piece movements
- **Multiple Controllers**:
  - PID with feedforward compensation and gravity cancellation
  - Model Predictive Control (MPC) for optimal constrained control
- **Visualization Tools**: Real-time robot state visualization and performance metrics

## Dependencies

- MATLAB R2023b or later
- Simulink
- Control System Toolbox
- Model Predictive Control Toolbox
- Simscape Multibody (optional, for 3D visualization)

## Setup and Usage

1. Clone this repository to your local machine
2. Open MATLAB and navigate to the project directory
3. Run the setup script to initialize workspace variables:
   ```matlab
   run scripts/setup.m
   ```
4. Open one of the Simulink models in the `models/` directory
5. Run simulations and analyze results

## Controller Design

### PID Controller

The PID controller implements a joint-space control law:

```
τ = Kp(qᵈ-q) + Ki∫(qᵈ-q)dt + Kd(q̇ᵈ-q̇) + M(q)q̈ᵈ + C(q,q̇)q̇ + G(q)
```

Where:
- τ: Joint torques
- q, q̇, q̈: Joint positions, velocities, and accelerations
- qᵈ, q̇ᵈ, q̈ᵈ: Desired joint positions, velocities, and accelerations
- Kp, Ki, Kd: Proportional, integral, and derivative gain matrices
- M(q): Inertia matrix
- C(q,q̇): Coriolis matrix
- G(q): Gravity vector

### MPC Controller

The MPC controller uses a linearized model of the system to predict future states and optimize control actions while respecting constraints:

- Prediction horizon: 10-20 steps
- Control horizon: 3-5 steps
- Constraints on joint positions, velocities, and torques
- Cost function balancing tracking performance and control effort

## Performance Metrics

The following metrics are used to evaluate controller performance:
- Tracking error (RMSE)
- Settling time
- Control effort
- Robustness to disturbances and parameter variations

## Example Usage

To run a simulation with the PID controller tracking a chess piece movement:

```matlab
% Load parameters
run scripts/setup.m

% Define chess move
start_square = 'e2';
end_square = 'e4';

% Set up simulation parameters
sim_time = 5;  % seconds
sample_time = 0.001;  % seconds

% Run simulation
open('models/manipulator_pid.slx');
set_param('manipulator_pid', 'StartTime', '0', 'StopTime', num2str(sim_time));
sim('manipulator_pid');

% Analyze results
analyze_results;
```

## License

This project is released under the MIT License. See LICENSE file for details.

## Acknowledgments

- The dynamics and kinematics models are based on robotic manipulator theory
- Chess coordinate system adapted for robotic manipulation applications