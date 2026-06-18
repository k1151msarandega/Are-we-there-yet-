# Are We There Yet?

A **car-like robot motion simulator** that models kinematic behavior based on input files describing vehicle dynamics and control inputs.

## Overview

This project simulates the path and state evolution of a vehicle (such as an autonomous car) over time. It reads configuration files specifying the vehicle's initial position, orientation, wheelbase, and a sequence of velocity and steering commands, then outputs the resulting trajectory.

## What It Does

- **Simulates vehicle kinematics** using a simple but accurate car-like robot model
- **Processes input files** containing initial pose, vehicle parameters, and control inputs (velocity and steering angle rate)
- **Outputs trajectory data** showing position, orientation, velocity, and steering angle at each timestep
- **Validates input** with detailed error reporting for malformed data

## Key Features

- Models a car's motion using kinematic equations with steering angle and velocity constraints
- Supports variable timestep simulation
- Produces CSV output compatible with MATLAB visualization
- Robust error handling and validation

## Quick Start

### Usage
```bash
./project2 inputFile outputFile
```

### Input File Format
```
Wheelbase: 2.6
InitialPose: 1,0,0,0
Dt: 0.100
1,0
1,0
1,0
```

Where:
- **Wheelbase**: Vehicle wheelbase length (L) in meters
- **InitialPose**: x, y, tire_angle (delta), heading (theta)
- **Dt**: Timestep for simulation
- **Following lines**: Velocity and steering angle rate pairs (v, deltaDot)

### Output File Format
CSV with columns: `time, x, y, theta, delta, v, deltaDot`

```
0.000,1.000,0.000,0.000,0.000,1.000,0.000
0.100,1.100,0.000,0.000,0.000,1.000,0.000
0.200,1.200,0.000,0.000,0.000,1.000,0.000
...
```

## Project Structure

The implementation uses object-oriented design with the following classes:

- **MathVector**: 1D vector of doubles with I/O validation
- **Input**: Vehicle control inputs (velocity, steering rate) - inherits from MathVector
- **State**: Vehicle state (position, orientation, steering angle) - inherits from MathVector
- **Vehicle**: Simulates car dynamics and maintains state
- **Director**: Orchestrates simulation loop using Vehicle and Input sequences
- **Project2**: Main controller and file I/O handler

## Kinematics Model

The vehicle motion follows these equations:

```
x(t+Dt) = x(t) + Dt * v(t) * cos(delta(t)) * cos(theta(t))
y(t+Dt) = y(t) + Dt * v(t) * cos(delta(t)) * sin(theta(t))
delta(t+Dt) = delta(t) + Dt * deltaDot(t)
theta(t+Dt) = theta(t) + Dt * v(t) * (1/L) * sin(delta(t))
```

Where:
- *x, y*: position
- *theta*: heading angle
- *delta*: steering/tire angle (constrained to ±0.5236 radians)
- *v*: velocity
- *deltaDot*: steering angle rate
- *L*: wheelbase

## Validation

A MATLAB visualization script is included to validate results by plotting the vehicle's path and animating the simulation.
