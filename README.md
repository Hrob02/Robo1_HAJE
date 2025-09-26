# Robo1_HAJE

**Robotics Studio 1**  
Development and simulated testing of an engineering prototype for a client working in forest and environmental management.

## Overview

This project simulates a drone-based system designed to survey a landscape and assess bushfire risk. By analyzing quantifiable characteristics of objects within the environment (e.g., size, density, vegetation type), the system can determine the likelihood of fire ignition or spread in specific areas.

The prototype is developed entirely in simulation, with a modular class-based architecture.

## Key Modules

- **Movement**: Controls the simulated drone's motion and position tracking.
- **Path Planning**: Generates efficient navigation paths across the terrain.
- **Object Detection**: Identifies and classifies environmental objects from the landscape.
- **Fire Risk Analysis**: Evaluates environmental data to assign a fire risk score to different regions.

## Project Scope

This prototype is intended to:

- Demonstrate how robotic systems can assist in landscape monitoring and fire risk analysis.
- Operate in a simulated environment using ROS 2 and Gazebo (or equivalent simulation tools).
- **Possible Extension**: Test different fire risk models based on the physical properties of surrounding objects.

## Future Applications

Potential future upgrades to this system include:

- **IoT Integration**: Connect to external weather APIs for live data (humidity, temperature, wind).
- **Real-World Deployment**: Adapt the simulation for hardware testing and integration with real drones or sensor networks.

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/Hrob02/Robo1_HAJE.git
