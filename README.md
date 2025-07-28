# PX4 TVC (Thrust Vector Control)

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases)

A modified PX4 autopilot implementation specifically designed for inverted coaxial drones with thrust vector control capabilities.

## Overview

This repository contains a specialized fork of PX4 autopilot software, customized for inverted coaxial drone configurations. The modifications enable precise control of coaxial motor systems with gimbal-based thrust vectoring.

**⚠️ Important:** This code is not compatible with standard PX4 source code and should only be used for inverted coaxial drone applications.

>**Note:** All git tag checks have been disbled in this code.

## Key Modifications

### 1. New Coaxial Airframe Support
- Custom airframe configuration for inverted coaxial drones
- Specialized control allocation for dual coaxial motors
- Gimbal servo integration for thrust vector control

### 2. Enhanced Gazebo ESC Interface
- Improved simulation accuracy for coaxial motor systems
- More intuitive source code logic
- Better alignment between simulation and real-world behavior

## Adding Custom Airframes

### Step 1: Define Airframe Parameters

Add your airframe parameters to `/src/modules/control_allocator/module.yaml`:

```yaml
parameters:
  - group: Geometry
    definitions:
      # TVC Motor Parameters
      CA_TVC_M${i}_CT:
        description:
          short: Thrust coefficient for TVC Motor ${i}
        type: float
        default: 0.1
        min: 0.0
        num_instances: 2
        instance_start: 0
      
      CA_TVC_M${i}_KMR:
        description:
          short: Roll moment coefficient for TVC Motor ${i}
        type: float
        default: 0.02
        num_instances: 2
        instance_start: 0
      
      # TVC Servo Parameters
      CA_TVC_S${i}_GAIN:
        description:
          short: Gain for TVC servo ${i}
        type: float
        default: 1.0
        min: 0.0
        max: 10.0
        decimal: 2
        num_instances: 2
        instance_start: 0
      
      CA_TVC_S${i}_MAX:
        description:
          short: Maximum output limit for TVC servo ${i} (normalized)
        type: float
        default: 1.0
        min: -1.0
        max: 1.0
        decimal: 2
        num_instances: 2
        instance_start: 0
```

### Step 2: Configure Airframe Type

Add the airframe configuration in the same `module.yaml` file:

```yaml
config:
  param: CA_AIRFRAME
  types:
    13: # Coaxial Gimbal Drone
      actuators:
        - actuator_type: 'motor'
          count: 2
          item_label_prefix: ['Motor Top', 'Motor Bottom']
          per_item_parameters:
            extra:
              - name: 'CA_TVC_M${i}_CT'
                label: "Thrust Coeff M${i}"
              - name: 'CA_TVC_M${i}_KMR'
                label: "Roll Moment Coeff M${i}"
        
        - actuator_type: 'servo'
          count: 2
          item_label_prefix: ['Gimbal Roll', 'Gimbal Pitch']
          per_item_parameters:
            extra:
              - name: 'CA_TVC_S${i}_GAIN'
                label: 'Gimbal Servo ${i} Gain'
              - name: 'CA_TVC_S${i}_MAX'
                label: 'Gimbal Servo ${i} Max Limit'
              - name: 'CA_TVC_S${i}_MIN'
                label: 'Gimbal Servo ${i} Min Limit'
              - name: 'CA_TVC_S${i}_GEO_A'
                label: 'Gimbal Servo ${i} Geometry A'
              - name: 'CA_TVC_S${i}_GEO_B'
                label: 'Gimbal Servo ${i} Geometry B'
              - name: 'CA_TVC_S${i}_GEO_C'
                label: 'Gimbal Servo ${i} Geometry C'
              - name: 'CA_TVC_S${i}_GEO_D'
                label: 'Gimbal Servo ${i} Geometry D'
              - name: 'CA_TVC_S${i}_GEO_E'
                label: 'Gimbal Servo ${i} Geometry E'
      parameters:
        - label: 'Distance between COM and gimbal center'
          name: CA_TVC_COM_DIS
```

### Step 3: Create Airframe Configuration File

Create a configuration file in `ROMFS/init.d-posix/airframes/` with the naming convention `<number>_<name>`:

```bash
#!/bin/sh
# @name TVC Coaxial Drone
# @type Coaxial
# @maintainer Your Name <email@example.com>

. ${R}etc/init.d/rc.mc_defaults

param set-default MAV_TYPE 9 # Rocket

# Simulator configuration
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=tvc}

param set-default SIM_GZ_EN 1

# Sensor configuration
param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 1

# Airframe configuration
param set-default CA_AIRFRAME 13
param set-default CA_ROTOR_COUNT 2
param set-default CA_SV_TL_COUNT 2

# Motor parameters
param set-default CA_TVC_M0_CT 0.0000048449
param set-default CA_TVC_M0_KMR 0.001
param set-default CA_TVC_M1_CT 0.0000048449
param set-default CA_TVC_M1_KMR -0.001

# Servo parameters
param set-default CA_TVC_S0_GAIN 1
param set-default CA_TVC_S0_MAX 1
param set-default CA_TVC_S0_MIN -1
param set-default CA_TVC_S0_GEO_A 0.05
param set-default CA_TVC_S0_GEO_B 0.01
param set-default CA_TVC_S0_GEO_C 0.06
param set-default CA_TVC_S0_GEO_D 0.02
param set-default CA_TVC_S0_GEO_E 0.06

param set-default CA_TVC_S1_GAIN 1
param set-default CA_TVC_S1_MAX 1
param set-default CA_TVC_S1_MIN -1
param set-default CA_TVC_S1_GEO_A 0.05
param set-default CA_TVC_S1_GEO_B 0.01
param set-default CA_TVC_S1_GEO_C 0.06
param set-default CA_TVC_S1_GEO_D 0.02
param set-default CA_TVC_S1_GEO_E 0.06

# Geometry parameters
param set-default CA_TVC_COM_DIS 0.275170

# Gazebo interface configuration
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_MIN1 0
param set-default SIM_GZ_EC_MIN2 0
param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_SV_FUNC1 201  # Roll control servo
param set-default SIM_GZ_SV_FUNC2 202  # Pitch control servo
```

**Note:** Remember to add the filename to `CMakeLists.txt` in the same directory.

### Step 4: Implement Control Allocation Logic

Create `ActuatorEffectivenessTVC.cpp` and `ActuatorEffectivenessTVC.hpp` files in `src/modules/control_allocator/ActuatorEffectiveness/`:

#### Key Functions to Implement:

**getEffectivenessMatrix()** - Override default PX4 control allocation by adding `matrix::Vector3f{}, matrix::Vector3f{}` when adding motor/servos:
```cpp
bool ActuatorEffectivenessTVC::getEffectivenessMatrix(Configuration &configuration,
                                EffectivenessUpdateReason external_update)
{
    if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !_geometry_updated) {
        return false;
    }

    // Add motors with zero vectors to override default allocation
    _motor1_idx = configuration.addActuator(ActuatorType::MOTORS, 
        matrix::Vector3f{}, matrix::Vector3f{}); 
    _motor2_idx = configuration.addActuator(ActuatorType::MOTORS,
        matrix::Vector3f{}, matrix::Vector3f{});  

    // Add servos
    _servo_roll_idx = configuration.addActuator(ActuatorType::SERVOS,
        matrix::Vector3f{}, matrix::Vector3f{});
    _servo_pitch_idx = configuration.addActuator(ActuatorType::SERVOS,
        matrix::Vector3f{}, matrix::Vector3f{});

    _geometry_updated = false;
    return true;
}
```

**updateSetpoint()** - Implement custom control allocation logic:
```cpp
void ActuatorEffectivenessTVC::updateSetpoint(
    const matrix::Vector<float, NUM_AXES> &control_sp, 
    int matrix_index, 
    ActuatorVector &actuator_sp, 
    const matrix::Vector<float, NUM_ACTUATORS> &actuator_min, 
    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
    // Your custom control allocation logic here
}
```

### Step 5: Integration

1. Include the new effectiveness file in `ControlAllocator.hpp`
2. Update the `update_effectiveness_source()` function in `ControlAllocator.cpp`

## Gazebo ESC Interface Modifications

Improvements have been made to `src/simulation/gz_bridge/GZMixingInterfaceESC.cpp` to enhance simulation accuracy and code logic.

**Reference:** [PX4 Discuss - Potential Bug in PX4 GZ Bridge ESC Interface](https://discuss.px4.io/t/potential-bug-in-px4-gz-bridge-esc-interface/46132)

## Build and Run Instructions

### Building the Project

```bash
make px4_sitl_default
```

### Running the TVC Airframe
First copy the model file from main repo to GZ simulation folder as mentioned in main repo README instructions.

```bash
# Use the airframe number defined in ROMFS/init.d-posix/airframes/
PX4_SYS_AUTOSTART=6002 ./build/px4_sitl_default/bin/px4
```

## Documentation References

- [PX4 Control Allocation Documentation](https://docs.px4.io/main/en/concept/control_allocation.html)
- [PX4 Airframe Configuration](https://docs.px4.io/main/en/dev_airframe/adding_a_new_frame.html)

## Contributing

When contributing to this project, please ensure that:

1. All modifications maintain compatibility with the inverted coaxial drone configuration
2. Simulation accuracy improvements are tested thoroughly
3. Documentation is updated to reflect any changes

## License

This project maintains the same license as the original PX4 project.