# Tutorial: EnergyPlus Simulation with PI Control and Night Setback

## Table of Contents
1. [Introduction to Building Control](#introduction-to-building-control)
2. [Control Hierarchy in Building Systems](#control-hierarchy-in-building-systems)
3. [PI Controllers and Temperature Control](#pi-controllers-and-temperature-control)
4. [Night Setback Strategies](#night-setback-strategies)
5. [Step-by-Step Code Walkthrough](#step-by-step-code-walkthrough)
6. [Key Performance Indicators](#key-performance-indicators)
7. [Practical Considerations](#practical-considerations)
8. [Advanced Topics](#advanced-topics)
9. [Exercises](#exercises)

## Introduction to Building Control

Building control is the process of regulating building systems to maintain desired conditions while optimizing energy consumption. This tutorial demonstrates how to implement closed-loop control for building thermal systems using EnergyPlus simulations.

### Key Concepts:

- **Control System**: A mechanism that regulates a process variable (temperature) to match a desired setpoint
- **Feedback Control**: Using measurements (actual temperature) to adjust control actions (heating power)
- **High-level vs Low-level Control**: High-level controller determines setpoints and distributes these to the low-level feedback controller
- **Simulation**: Using EnergyPlus to model building physics and test control strategies
- **Energy Efficiency**: Balancing comfort requirements with minimal energy consumption

## Control Hierarchy in Building Systems

Modern building control typically implements a hierarchical structure:

### High-Level Control:
- Determines setpoints based on schedules, occupancy, or optimization
- Operates on longer time scales (hours to days)
- Examples: Night setback, demand response, model predictive control
- Focuses on energy efficiency and system-wide coordination

### Low-Level Control:
- Regulates actuators to achieve setpoints provided by high-level control
- Operates on shorter time scales (seconds to minutes)
- Examples: PI/PID controllers, on-off control with hysteresis
- Focuses on stability, disturbance rejection, and tracking performance

This hierarchy allows separation of concerns:
1. High-level controllers optimize operation without dealing with detailed dynamics
2. Low-level controllers handle physical constraints and dynamics without needing to consider optimization objectives

## PI Controllers and Temperature Control

Proportional-Integral (PI) controllers are widely used in building systems due to their simplicity and effectiveness.

### Controller Structure:

The PI controller calculates a control signal (u) based on the error (e) between setpoint and measured temperature:

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt
```

Where:
- e(t) = setpoint - measured_temperature
- Kp: Proportional gain (immediate response to current error)
- Ki: Integral gain (removes steady-state error), often expressed as Ki = Kp/tauI
- tauI: Integral time constant

### Key Parameters:

- **Kp (Proportional Gain)**: Determines how aggressively the controller responds to errors
  - Higher values: faster response but risk of oscillation
  - Lower values: slower response but more stable
  
- **tauI (Integral Time Constant)**: Determines how quickly past errors are integrated
  - Smaller values: faster elimination of steady-state error but more overshoot
  - Larger values: slower integration but more stable
  
- **Anti-Windup**: Prevents integral term from accumulating when actuators saturate
  - Important for systems with physical limitations (like maximum heating power)

For building thermal control:
- Typical Kp values: 0.1 to 2.0
- Typical tauI values: 600 to 3600 seconds (10 to 60 minutes)
- The slow thermal dynamics of buildings usually require conservative tuning

## Night Setback Strategies

Night setback is an energy-saving strategy that reduces temperature setpoints during unoccupied or low-activity periods.

## Step-by-Step Code Walkthrough

Let's examine each section of the provided example code:

### Step 0: Installation and File Preparation

#### 0.1 Install the Library

Before using the `epluscontrol` library, you need to install it:

```bash
# Install the library
pip install epluscontrol
```

**Explanation:**
- This step is performed once in your environment, not as part of the script
- You may want to use a virtual environment to manage dependencies
- Check the library documentation for specific version requirements

#### 0.2 Download Required Files

You'll need to download the example files used in this tutorial:

```
Eplus_project_room.idf - EnergyPlus input data file containing the building model
COPENHAGEN.epw - EnergyPlus weather file for Copenhagen
```

**Where to find the files:**
- The example files are available in the GitHub repository: [github.com/michael666nan/epluscontrol/tree/master/tutorial_data/](https://github.com/michael666nan/epluscontrol/tree/master/tutorial_data/)
- Download these files to your working directory before running the script
- Alternatively, you can use the following command to download them directly from GitHub:

```python
# Optional: Download example files directly
import requests
import os

# Create directory for files if it doesn't exist
os.makedirs("example_files", exist_ok=True)

# File URLs from the GitHub repository (update these with the correct URLs)
files = {
    "Eplus_project_room.idf": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/Eplus_project_room.idf",
    "COPENHAGEN.epw": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/COPENHAGEN.epw"
}

# Download each file
for filename, url in files.items():
    try:
        response = requests.get(url)
        response.raise_for_status()  # Check for HTTP errors
        
        with open(os.path.join("example_files", filename), "wb") as f:
            f.write(response.content)
        
        print(f"Downloaded {filename} successfully!")
    except Exception as e:
        print(f"Error downloading {filename}: {e}")
```

**Note:** Make sure to update the GitHub URLs with your actual repository path.

### Step 1: Importing the Library

```python
"""
=======================================================================
Example: EnergyPlus Simulation with PI Control and Night Setback
=======================================================================

Description:
This example demonstrates how to set up a basic closed-loop heating
control simulation using EnergyPlus. It shows implementation of:
  - PI controller for low-level temperature control
  - Night setback strategy for energy-efficient setpoint scheduling
  - Basic sensor configuration and result visualization
  - Analysis of energy consumption and temperature violations

Prerequisites:
  - EnergyPlus V24-2-0 installed
  - epluscontrol package installed
  - Basic understanding of building HVAC control concepts

=======================================================================
"""
import epluscontrol as epc
```

**Explanation:**
- The `epluscontrol` library provides tools for implementing control strategies with EnergyPlus
- This single import gives access to all the necessary components: 
  - Simulation manager
  - Controllers
  - Utilities
  - Visualization tools

### Step 2: Setting up the Simulation

```python
# Initialize the simulation manager
# This creates the main interface for controlling the EnergyPlus simulation
# - idf: Path to the EnergyPlus input data file containing building model
# - epw: Path to the EnergyPlus weather file for the location (Copenhagen)
# - energyplus_dir: Directory where EnergyPlus is installed
eplusman = epc.EPManager(
    idf="example_files/Eplus_project_room.idf",  # energyplus model file (from downloaded files)
    epw="example_files/COPENHAGEN.epw",          # weather file (from downloaded files)
    energyplus_dir="C:\EnergyPlusV24-2-0"        # energyplus directory
    )

# Configure simulation period
# This sets the start and end dates for the simulation
# For this example, we're simulating the first two weeks of January
eplusman.set_run_period(
    start_month=1,    # January
    start_day=1,      # Day 1
    end_month=1,      # January
    end_day=14)       # Day 14 (2 weeks total)

# Set maximum radiator heat output (W)
# This limits how much heating power the radiator can provide
# Adjust this based on your building's heating system capacity
eplusman.set_max_power(1000)  # 1000 Watts maximum power
```

**Explanation:**
- `EPManager` is the central class that coordinates the simulation
- The `idf` file contains the building geometry, materials, and systems
- The `epw` file contains weather data for the simulation location
- Setting the simulation period allows for focused analysis of specific times
- The maximum power setting prevents unrealistic control actions

### Step 3: Defining Sensors

```python
# Sensors collect data from the simulation
# Each sensor needs:
# - variable: The EnergyPlus output variable name
# - key: The specific object to measure (zone, surface, etc.)
# - unit: The unit of measurement for display purposes
SENSOR_CONFIG = {
    "Indoor Temp": {
        "variable": "ZONE MEAN AIR TEMPERATURE",  # Zone air temperature
        "key": "ZONE 1",                          # For the specific zone named "ZONE 1"
        "unit": "°C"                              # Temperature in Celsius
    },
    "Outdoor Temp": {
        "variable": "SITE OUTDOOR AIR DRYBULB TEMPERATURE",  # Outdoor temperature
        "key": "ENVIRONMENT",                                # Environment object
        "unit": "°C"
    },
    "Solar Gain": {
        "variable": "Surface Outside Face Incident Solar Radiation Rate per Area",  # Solar radiation
        "key": "East Wall",                                                         # On the east wall
        "unit": "W/m2"                                                              # Watts per square meter
    },
    "Heat Power": {
        "variable": "Electric Equipment Electricity Rate",  # Electricity consumption rate
        "key": "RADIATOR",                                  # Of the radiator
        "unit": "W",                                        # Watts
    }
}
# Add Sensors to the Simulation Manager
# This tells the simulation manager which variables to track during the simulation
eplusman.set_sensors(SENSOR_CONFIG)
```

**Explanation:**
- Sensors define the variables to extract from the EnergyPlus simulation
- Each sensor has three components:
  - `variable`: The EnergyPlus output variable name (must match EnergyPlus naming)
  - `key`: The specific object to measure (zone, surface, equipment, etc.)
  - `unit`: The unit for display purposes
- These sensors provide:
  - Feedback for controllers (Indoor Temp)
  - Disturbance measurements (Outdoor Temp, Solar Gain)
  - Performance monitoring (Heat Power)
- For available variables, consult the EnergyPlus Input-Output Reference

### Step 4: Adding Low-Level Control

```python
# The PI controller implements Proportional-Integral control
# It directly controls the heating output to maintain the desired temperature
# Parameters:
# - time_step: How often the controller executes (in seconds)
# - sensor_name: Which sensor to read for feedback
# - Kp: Proportional gain - how strongly to respond to current error
# - tauI: Integral time constant - how strongly to respond to accumulated error
# - with_antiwindup: Prevents integral term from growing too large when system can't reach setpoint
pi = epc.low_level_control.PIController(
    time_step=1,                  # Update control signal every 1 second
    sensor_name="Indoor Temp",    # Control based on indoor temperature
    Kp=0.5,                       # Proportional gain coefficient
    tauI=1800.0,                  # Integral time constant (30 minutes)
    with_antiwindup=True          # Prevent integral windup
    )

# Add the Controller to the Simulation Manager
# This connects the PI controller to the simulation
eplusman.set_low_level_control(pi)
```

**Explanation:**
- The PI controller implements classical feedback control
- `time_step=1` means the controller updates every second (frequent updates for smooth control)
- `sensor_name="Indoor Temp"` specifies which measurement to use for feedback
- `Kp=0.5` is the proportional gain (moderate response to avoid oscillations)
- `tauI=1800.0` is the integral time constant (30 minutes, relatively slow integration)
- `with_antiwindup=True` prevents integral term issues when the actuator saturates
- This controller takes a setpoint from the high-level controller and maintains it

### Step 5: Adding High-Level Control

```python
# The Night Setback controller implements an energy-saving strategy
# It reduces the temperature setpoint during nighttime hours
# Parameters:
# - time_step: How often to update the setpoint (in seconds)
# - sensor_name: Which sensor to monitor (typically just for reference)
# - day_setpoint: Desired temperature during daytime (°C)
# - night_setpoint: Reduced temperature during nighttime (°C)
night = epc.high_level_control.NightSetback(
    time_step=60,              # Update setpoint every minute
    sensor_name="Indoor Temp", # Reference sensor (not used for control)
    day_setpoint=23,           # 23°C during daytime
    night_setpoint=18          # 18°C during nighttime (energy saving)
    )

# Add the controller to the Simulation Manager
# This connects the Night Setback controller to the simulation
eplusman.set_high_level_control(night)
```

**Explanation:**
- The Night Setback controller implements a simple energy-saving strategy
- `time_step=60` means setpoints update every minute (less frequent than PI controller)
- `day_setpoint=23` sets the daytime temperature target (23°C)
- `night_setpoint=18` sets the nighttime temperature target (18°C)
- This 5°C setback is typical for residential and commercial buildings
- By default, the night setback uses predefined periods (e.g., 10 PM to 6 AM)
- The high-level controller provides setpoints to the low-level controller

### Step 6: Running the Simulation

```python
# Execute the simulation and store results in a pandas DataFrame
# This runs EnergyPlus with all the configured sensors and controllers
df = eplusman.simulate()

# Print the column names available in the results dataframe
# This shows what data was collected during the simulation
print(df.columns)   # Print columns stored in dataframe
```

**Explanation:**
- `simulate()` executes the EnergyPlus simulation with the configured controllers
- EnergyPlus runs as a subprocess, with `epluscontrol` managing the communication
- Results are stored in a pandas DataFrame for easy analysis
- The dataframe contains:
  - Sensor readings (Indoor Temp, Outdoor Temp, etc.)
  - Control signals (heating power)
  - Setpoints (day/night values)
  - Timestamps as the index
- Printing columns helps understand what data is available for analysis

### Step 7: Visualizing Results

```python
# Generate standard plots showing simulation results
# This typically includes temperature profiles, setpoints, and power usage
eplusman.plot()  # This creates a default set of plots for visualization
```

**Explanation:**
- `plot()` creates a standard set of visualizations for quick analysis
- The plots typically include:
  - Indoor temperature vs. setpoint over time
  - Control signal (heating power) over time
  - Outdoor temperature and solar gain (disturbances)
  - Multiple subplots with synchronized x-axis (time)
- These plots help evaluate controller performance and system behavior

### Step 8: Calculating Key Performance Indicators

```python
# Calculate the total energy consumption in kilowatt-hours
# This sums up the power usage over time and converts to kWh
tot_energy = epc.utils.total_energy(df['Heat Power'])
print(f"Total energy consumption: {tot_energy} kWh")

# Calculate temperature violations (when room temperature falls below setpoint)
# This measures how well the system maintained the desired temperature
# The result is in Kelvin-hours (K·h), representing the magnitude and duration of violations
below_20 = epc.utils.temperature_violations(df['Indoor Temp'], df['setpoint'], type="lower_bound")
print(f"Kelvin hours below setpoint: {below_20} K·h")
```

**Explanation:**
- Key Performance Indicators (KPIs) quantify control performance
- `total_energy()` calculates the cumulative energy consumption in kilowatt-hours
  - This is a key metric for energy efficiency
- `temperature_violations()` measures comfort violations in Kelvin-hours
  - It integrates the magnitude and duration of temperature deviations
  - `type="lower_bound"` counts when temperature falls below setpoint
  - The unit K·h (Kelvin-hours) represents the severity of violations
- These metrics allow objective comparison between different control strategies

## Key Performance Indicators

When evaluating building control strategies, several key performance indicators (KPIs) are commonly used:

### 1. Energy Consumption
- **Total Energy Use**: Overall energy consumption during the test period
- **Peak Power Demand**: Maximum power requirement
- **Energy Use Intensity**: Energy consumption normalized by floor area

### 2. Thermal Comfort
- **Temperature Violations**: Time and magnitude of deviations from setpoint
- **Predicted Mean Vote (PMV)**: Thermal comfort index (-3 to +3 scale)
- **Predicted Percentage Dissatisfied (PPD)**: Percentage of occupants expected to be dissatisfied

### 3. Control Performance
- **Setpoint Tracking Error**: Average deviation from setpoint
- **Settling Time**: Time required to reach steady state after disturbances
- **Control Effort**: Frequency and magnitude of control signal changes

### 4. Specialized Metrics
- **Kelvin-hours**: Integrated temperature deviation over time
  - Formula: ∑(|Tactual - Tsetpoint|) × Δt, when violation occurs
  - Units: K·h (Kelvin-hours)
  - Interpretation: Lower values indicate better comfort maintenance
  - Example: 10 K·h could mean 1K deviation for 10 hours or 2K for 5 hours

## Practical Considerations

### Tuning PI Controllers
- **Start Conservative**: Begin with low Kp and high tauI
- **Increase Proportional Gain**: Gradually increase Kp until response is acceptable
- **Decrease Integral Time**: Gradually decrease tauI to eliminate steady-state error
- **Test with Disturbances**: Ensure good performance during weather changes
- **Check for Oscillations**: Reduce gains if temperature oscillates

### Night Setback Configuration
- **Recovery Time**: Schedule setpoint increase early enough before occupancy
- **Setback Depth**: Balance between energy savings and recovery capability
- **Building Thermal Mass**: High thermal mass requires earlier recovery
- **Minimum Temperature**: Ensure setback temperature prevents moisture issues

### Computational Efficiency
- **Control Time Steps**: Balance between control precision and computational load
- **Simulation Duration**: Select representative periods for testing
- **Variable Time Steps**: Consider using smaller steps during transitions

## Advanced Topics

### Model Predictive Control (MPC)
- Uses building model to optimize control strategies
- Anticipates future conditions using weather forecasts
- Minimizes energy while maintaining comfort
- Implementation using the library:
  ```python
  mpc = epc.high_level_control.ModelPredictiveController(
      time_step=300,            # 5-minute planning
      prediction_horizon=24,    # 24-hour lookahead
      building_model=model,     # Thermal model of building
      objective="min_energy"    # Optimization objective
  )
  ```

### Occupancy-Based Control
- Adjusts setpoints based on occupancy detection
- Reduces energy use during unoccupied periods
- Can be implemented using:
  ```python
  occ = epc.high_level_control.OccupancyController(
      time_step=60,
      occupied_setpoint=22,
      unoccupied_setpoint=18,
      occupancy_schedule=schedule
  )
  ```

### Multi-Zone Control
- Coordinates control across multiple thermal zones
- Accounts for thermal interactions between zones
- Prioritizes zones based on occupancy or importance

### Weather-Predictive Control
- Incorporates weather forecasts into control decisions
- Pre-cools before hot periods or pre-heats before cold periods
- Optimizes for renewable energy availability

## Exercises

1. **Controller Tuning**: Modify the PI controller parameters (Kp and tauI) and observe the effects on temperature control and energy consumption. Create a table comparing different parameter sets.

2. **Setback Analysis**: Experiment with different day/night setpoint combinations to find an optimal balance between energy savings and comfort. Plot energy consumption vs. comfort violation metrics for various settings.

3. **Disturbance Response**: Implement a rapid outdoor temperature change scenario and compare how different control strategies respond to this disturbance.

4. **Advanced Comfort Metric**: Extend the code to calculate Predicted Mean Vote (PMV) and Predicted Percentage Dissatisfied (PPD) metrics using temperature, humidity, and other factors.

5. **Custom Control Strategy**: Implement a custom high-level controller that modifies setpoints based on outdoor temperature (e.g., weather-compensation curve). Compare its performance with the basic night setback.

```python
# Example exercise implementation: Weather-compensated control
class WeatherCompensatedControl(epc.high_level_control.BaseController):
    def __init__(self, time_step, outdoor_sensor, max_setpoint=23, min_setpoint=19):
        super().__init__(time_step)
        self.outdoor_sensor = outdoor_sensor
        self.max_setpoint = max_setpoint
        self.min_setpoint = min_setpoint
        
    def calculate_setpoint(self, current_time, sensor_readings):
        outdoor_temp = sensor_readings[self.outdoor_sensor]
        # Linear compensation: lower setpoint when warmer outside
        if outdoor_temp < 0:
            return self.max_setpoint
        elif outdoor_temp > 20:
            return self.min_setpoint
        else:
            # Linear interpolation between limits
            return self.max_setpoint - (outdoor_temp / 20) * (self.max_setpoint - self.min_setpoint)
```