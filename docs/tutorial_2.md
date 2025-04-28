# Tutorial: EnergyPlus Simulation with Model Predictive Control

## Table of Contents
1. [Introduction to Model Predictive Control](#introduction-to-model-predictive-control)
2. [MPC in Building Systems](#mpc-in-building-systems)
3. [Direct vs. Indirect MPC](#direct-vs-indirect-mpc)
4. [Building Thermal Models](#building-thermal-models)
5. [Step-by-Step Code Walkthrough](#step-by-step-code-walkthrough)
6. [Key Performance Indicators](#key-performance-indicators)
7. [Practical Considerations](#practical-considerations)
8. [Advanced Topics](#advanced-topics)
9. [Exercises](#exercises)

## Introduction to Model Predictive Control

Model Predictive Control (MPC) is an advanced control strategy that uses a mathematical model of a system to predict its future behavior and optimize control actions over a prediction horizon. Unlike reactive controllers (such as PI or PID), MPC is proactive, making decisions based on anticipated future conditions.

### Key MPC Concepts:

- **Prediction Model**: Mathematical representation of building thermal dynamics
- **Prediction Horizon**: Future time window for which the controller optimizes (e.g., 24-48 hours)
- **Receding Horizon**: Only the first control action is implemented, then optimization is repeated
- **Cost Function**: Mathematical expression of control objectives (energy minimization, comfort maximization)
- **Constraints**: Limitations on temperature ranges, equipment capacities, and other operational boundaries

## MPC in Building Systems

Model Predictive Control offers several advantages for building control:

### Benefits of MPC:
- **Anticipatory Control**: Preheats or precools based on future weather and occupancy
- **Multi-objective Optimization**: Balances competing goals like comfort and energy efficiency
- **Constraint Handling**: Explicitly respects equipment limitations and comfort requirements
- **Energy Cost Reduction**: Can optimize based on time-varying electricity prices
- **Renewable Integration**: Can adjust consumption to match renewable energy availability

### MPC in Hierarchical Control:
MPC typically operates as a high-level controller that provides optimal setpoints to low-level controllers (like PI controllers) that actually operate the equipment.

## Direct vs. Indirect MPC

There are two primary approaches to implementing MPC in building systems:

### Indirect MPC:
- Generates temperature setpoints for low-level controllers
- Low-level controllers (e.g., PI) track these setpoints by manipulating heating/cooling equipment
- More common in practice due to compatibility with existing building control infrastructure
- Provides a layer of separation between optimization and physical actuation

### Direct MPC:
- Directly manipulates heating/cooling equipment power levels
- Eliminates the need for separate low-level controllers
- Can potentially achieve more precise control
- More susceptible to model mismatch issues
- Requires more detailed system modeling and faster computation

## Building Thermal Models

MPC effectiveness depends heavily on the quality of the prediction model used:

### Model Types:
- **Physics-based Models**: Based on thermodynamic principles and building characteristics
- **Data-driven Models**: Derived from historical building operation data
- **State-Space Models**: Represent building dynamics as differential equations
- **Grey-box Models**: Combine physical insights with data-driven parameter estimation

### Model Structure:
A typical state-space thermal model has the form:
```
x(k+1) = A·x(k) + B·u(k) + E·d(k)
y(k) = C·x(k) + D·u(k)
```
Where:
- x: State vector (e.g., temperatures of building elements)
- u: Control inputs (e.g., heating power)
- d: Disturbances (e.g., outdoor temperature, solar gains)
- y: Outputs (e.g., indoor temperature)
- A, B, C, D, E: Model matrices

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
Eplus_Project_Room.idf - EnergyPlus input data file containing the building model
COPENHAGEN.epw - EnergyPlus weather file for Copenhagen
second_order.npz - Pre-trained building thermal model
weather.csv - Weather forecast data
grid.csv - Electricity price forecast data
```

**Where to find the files:**
- The example files are available in the GitHub repository: [github.com/michael666nan/epluscontrol/tree/master/tutorial_data/](https://github.com/michael666nan/epluscontrol/tree/master/tutorial_data/)
- Download these files to your working directory before running the script
- Alternatively, you can use the following Python script to download them directly:

```python
# Download required files
import requests
import os

# File URLs from the GitHub repository (update these with the correct URLs)
files = {
    "Eplus_Project_Room.idf": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/Eplus_Project_Room.idf",
    "COPENHAGEN.epw": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/COPENHAGEN.epw",
    "second_order.npz": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/second_order.npz",
    "weather.csv": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/weather.csv",
    "grid.csv": "https://raw.githubusercontent.com/michael666nan/epluscontrol/master/tutorial_data/grid.csv"
}

# Download each file
for filename, url in files.items():
    try:
        print(f"Downloading {filename}...")
        response = requests.get(url)
        response.raise_for_status()  # Check for HTTP errors
        
        with open(filename, "wb") as f:  # Save directly to working directory
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
Example: EnergyPlus Simulation with Model Predictive Control
=======================================================================

Description:
This example demonstrates how to implement Model Predictive Control (MPC)
with EnergyPlus to optimize heating control in a building model.

MPC uses a mathematical model of the building to predict future behavior and
optimize control actions over a prediction horizon. This approach can achieve:
  - Improved thermal comfort
  - Reduced energy consumption
  - Lower operational costs
  - Optimization based on variable energy prices

This example shows both direct and indirect MPC implementation:
  - Direct MPC: Controller directly manipulates the heating power output
  - Indirect MPC: Controller sets temperature setpoints for a low-level controller

Prerequisites:
  - EnergyPlus V24-2-0 installed
  - epluscontrol package installed
  - Building model (IDF file) and weather data (EPW file)
  - Pretrained building thermal model (second_order.npz)

=======================================================================
"""
import epluscontrol as epc
```

**Explanation:**
- The `epluscontrol` library provides the tools for implementing MPC with EnergyPlus
- This import gives access to all necessary components: simulation manager, controllers, models, and utilities
- The docstring explains the purpose and benefits of MPC in building applications

### Step 2: Setting up the Simulation

```python
# Initialize the simulation manager
# This component orchestrates the co-simulation between our Python control
# algorithms and the EnergyPlus building physics engine
eplusman = epc.EPManager(
    idf="Eplus_project_room.idf",           # Building model definition
    epw="COPENHAGEN.epw",                   # Weather data for Copenhagen
    energyplus_dir="C:\EnergyPlusV24-2-0"   # Path to EnergyPlus installation
    )

# Configure simulation period
# We'll simulate a one-week period in winter (January 1-7)
# This is sufficient to demonstrate the controller's performance
# while keeping computation time reasonable
eplusman.set_run_period(
    start_month=1,    # January
    start_day=1,      # Starting from the 1st
    end_month=1,      # January
    end_day=7)        # Ending on the 7th (one week)

# Set maximum heating capacity
# This defines the upper limit of heating power available from the radiator
# This constraint is crucial for realistic MPC optimization
eplusman.set_max_power(1000)  # 1000 Watts maximum heating power
```

**Explanation:**
- `EPManager` coordinates the co-simulation between Python and EnergyPlus
- The `idf` file contains building geometry, materials, and systems definition
- The `epw` file contains hourly weather data for the simulation location
- The simulation period is set to one week in January (winter conditions)
- Maximum power constraint (1000W) ensures realistic optimization

### Step 3: Configuring Sensors

```python
# Sensors connect the controller to the simulated building data
# These measurements are what would be provided by real sensors in practice
# Each sensor requires:
#   - variable: The specific EnergyPlus output variable to monitor
#   - key: The specific object where the measurement occurs
#   - unit: The measurement unit (for display purposes)
SENSOR_CONFIG = {
    "Indoor Temp": {
        "variable": "ZONE MEAN AIR TEMPERATURE",  # Indoor air temperature
        "key": "ZONE 1",                          # Primary thermal zone
        "unit": "°C"                              # Celsius
    },
    "Outdoor Temp": {
        "variable": "SITE OUTDOOR AIR DRYBULB TEMPERATURE",  # Weather condition
        "key": "ENVIRONMENT",                                # From weather file
        "unit": "°C"
    },
    "Solar Gain": {
        "variable": "Surface Outside Face Incident Solar Radiation Rate per Area",  # Solar radiation
        "key": "East Wall",                                                         # East-facing wall
        "unit": "W/m2"                                                              # Solar intensity measurement
    },
    "Heat Power": {
        "variable": "Electric Equipment Electricity Rate",  # Power consumption
        "key": "RADIATOR",                                  # Heating element
        "unit": "W",                                        # Watts
    }
}
# Register sensors with the simulation manager
# This tells EnergyPlus which variables to expose during co-simulation
eplusman.set_sensors(SENSOR_CONFIG)
```

**Explanation:**
- Sensors collect data from the EnergyPlus simulation
- `Indoor Temp` provides the primary controlled variable
- `Outdoor Temp` and `Solar Gain` represent major disturbances
- `Heat Power` monitors energy consumption
- Each sensor specifies an EnergyPlus variable name, object key, and unit
- These measurements feed into both the controller and performance evaluation

### Step 4: Configuring Low-Level Controller

```python
# The PI controller implements classic feedback control
# It works to maintain the temperature setpoint provided by the MPC
# by adjusting the actual heating power in real-time
# 
# Parameters:
#   - time_step: Control loop frequency
#   - sensor_name: Temperature measurement for feedback
#   - Kp: Proportional gain - response intensity to current error
#   - tauI: Integral time constant - response to accumulated error
#   - with_antiwindup: Prevents control saturation issues
pi = epc.low_level_control.PIController(
    time_step=1,                  # Fast response (1 second updates)
    sensor_name="Indoor Temp",    # Uses indoor temperature as feedback
    Kp=0.5,                       # Moderate proportional response
    tauI=1800.0,                  # 30-minute integral time constant
    with_antiwindup=True          # Prevents integral term accumulation when saturated
    )

# Register the PI controller with the simulation manager
# This connects the controller to the simulation loop
eplusman.set_low_level_control(pi)
```

**Explanation:**
- For indirect MPC, a low-level PI controller tracks setpoints from MPC
- `time_step=1` provides fast updates (every second)
- `Kp=0.5` gives moderate proportional response to avoid oscillations
- `tauI=1800.0` (30 minutes) provides gradual integration of steady-state errors
- `with_antiwindup=True` prevents integral term issues when the actuator saturates
- This controller layer handles disturbance rejection and setpoint tracking

### Step 5: Configuring Model Predictive Controller

```python
# Load the pre-trained building thermal model
# This state-space model captures the thermal dynamics of the building
# and allows the MPC to predict future temperature responses
model = epc.StateSpace.load("second_order.npz")

# Configure weather prediction parser
# In a real application, this would connect to a weather forecast API
# Here we use pre-generated forecast data from a CSV file
weather_parser = epc.parser.WeatherParser(filepath="weather.csv")

# Configure energy price prediction parser
# This provides time-varying electricity prices for cost optimization
# In practice, this could connect to market price data or forecasts
grid_parser = epc.parser.GridParser(filepath="grid.csv")

# Define comfort constraints (lower and upper temperature bounds)
# The MPC will try to keep the temperature within these bounds
low_setpoint_parser = epc.parser.SetpointParser(
    day_setpoint=20,      # Minimum comfortable temperature (day)
    night_setpoint=20,    # Minimum comfortable temperature (night)
    day_start_hour=6,     # Beginning of day period
    day_end_hour=19)      # End of day period

high_setpoint_parser = epc.parser.SetpointParser(
    day_setpoint=24,      # Maximum comfortable temperature (day)
    night_setpoint=24,    # Maximum comfortable temperature (night)
    day_start_hour=6,     # Beginning of day period
    day_end_hour=19)      # End of day period

# Create the MPC controller
# This is the core optimization-based controller that will determine 
# optimal heating strategies based on predictions and constraints
mpc = epc.high_level_control.ModelPredictiveControl(
    time_step=60,                   # Execute MPC every 60 seconds (1 minute)
    sensor_name="Indoor Temp",      # Temperature measurement for feedback
    model=model,                    # Building thermal model for predictions
    weather_parser=weather_parser,  # Weather forecast provider
    grid_parser=grid_parser,        # Energy price forecast provider
    low_setpoint_parser=low_setpoint_parser,    # Minimum comfort constraints
    high_setpoint_parser=high_setpoint_parser,  # Maximum comfort constraints
    max_power=eplusman.MAX_POWER,   # Maximum heating power constraint
    horizon=48,                     # Prediction horizon (48 timesteps ahead)
    slack_weight=10e5,              # Penalty for constraint violations
    solver="CLARABEL",              # Optimization solver to use
    print_output=True,              # Show solver details for debugging
    control_type='indirect',        # Use indirect MPC (setpoint control)
    correct_setpoint=True           # Apply feedback correction for model mismatch
    )

# Register the MPC controller with the simulation manager
# This establishes the MPC as the high-level decision maker
eplusman.set_high_level_control(mpc)
```

**Explanation:**
- First, a pre-trained state-space model is loaded for predictions
- Input data parsers are initialized:
  - `WeatherParser`: Provides weather forecast data
  - `GridParser`: Provides electricity price forecasts
  - `SetpointParser`: Defines comfort bounds (20-24°C)
- The MPC controller is configured with:
  - `time_step=60`: Updates every minute
  - `model`: The thermal model for predictions
  - `horizon=48`: Optimizes over 48 timesteps ahead
  - `slack_weight=10e5`: High penalty for comfort violations
  - `control_type='indirect'`: Generates setpoints for the PI controller
  - `correct_setpoint=True`: Uses feedback to correct for model inaccuracies
- The MPC is registered as the high-level controller in the simulation

### Step 6: Running the Simulation

```python
# Run the full co-simulation between EnergyPlus and our controllers
# Results are stored in a pandas DataFrame for analysis
df = eplusman.simulate()

# Add comfort bounds to results for visualization and analysis
df['lower_setpoint'] = low_setpoint_parser.get_values(df.index)
df['upper_setpoint'] = high_setpoint_parser.get_values(df.index)

# Add electricity pricing data to results
# This allows for economic performance evaluation
(df['price'], df['co2']) = grid_parser.get_values(df.index)

# Display the available data columns for reference
print("Available data columns:", df.columns)
```

**Explanation:**
- `simulate()` runs the co-simulation with EnergyPlus and returns results
- The simulation connects EnergyPlus with the controller hierarchy:
  - MPC provides optimized setpoints
  - PI controller tracks these setpoints
  - EnergyPlus simulates the building's thermal response
- Results are stored in a pandas DataFrame for analysis
- Additional columns are added to the results:
  - Comfort bounds (lower and upper setpoints)
  - Electricity prices
  - CO2 intensity of electricity
- This enriched dataset supports comprehensive performance evaluation

### Step 7: Visualizing Results

```python
# Generate standard visualization of simulation outcomes
# This includes temperature profiles, setpoints, and heating power
eplusman.plot()  # Default plots from the simulation manager
```

**Explanation:**
- The `plot()` method generates standard visualizations:
  - Indoor temperature vs. setpoint
  - Heating power over time
  - Outdoor conditions (temperature, solar radiation)
- These plots provide a quick overview of controller performance
- They show how well the controller maintained temperature within bounds
- They also show the patterns of energy usage that the MPC optimized

### Step 8: Evaluating Performance Metrics

```python
# Calculate total energy consumption
# This sums up all heating energy used during the simulation period
tot_energy = epc.utils.total_energy(df['Heat Power'])
print(f"Total energy consumption: {tot_energy} kWh")

# Calculate total energy cost
# This combines energy consumption with time-varying prices
tot_energy_cost = epc.utils.total_energy_cost(df['Heat Power'], df['price'])
print(f"Total energy costs: {tot_energy_cost} DKK")

# Calculate comfort violations (lower bound)
# This measures when room temperature falls below the minimum comfort level
# The result is in Kelvin-hours (K·h), representing magnitude and duration
below = epc.utils.temperature_violations(df['Indoor Temp'], df['lower_setpoint'], type="lower_bound")
print(f"Comfort violation (too cold): {below} K·h")

# Calculate comfort violations (upper bound)
# This measures when room temperature exceeds the maximum comfort level
above = epc.utils.temperature_violations(df['Indoor Temp'], df['upper_setpoint'], type="upper_bound")
print(f"Comfort violation (too hot): {above} K·h")
```

**Explanation:**
- Key Performance Indicators (KPIs) quantify controller effectiveness:
  - **Energy Consumption (kWh)**: Total heating energy used
  - **Energy Cost (DKK)**: Total cost based on time-varying prices
  - **Comfort Violations (K·h)**: Magnitude and duration of temperature constraint violations
- These metrics provide objective assessment of controller performance
- They allow quantitative comparison with other control strategies
- The balance between energy metrics and comfort metrics reveals the controller's priorities

### Step 9: Advanced Visualization

```python
# Generate comprehensive smart building visualization
# This shows the interrelationships between various system variables
_ = epc.utils.plot_smart_building_data(df)

# Create interactive MPC visualization tool
# This displays historical data and predictions at each MPC timestep
visualizer = epc.utils.MPCVisualizer(mpc)
# To launch the interactive visualizer, call:
# visualizer.run()
```

**Explanation:**
- `plot_smart_building_data()` creates an enhanced visualization:
  - Shows correlations between variables
  - Highlights energy usage in context of conditions
  - Provides a comprehensive view of system operation
- `MPCVisualizer` creates an interactive tool to explore MPC operation:
  - Shows predictions made at each timestep
  - Displays optimization constraints
  - Helps understand the controller's decision-making process
  - Useful for debugging and explanation

## Key Performance Indicators

When evaluating MPC performance, several key metrics are important:

### 1. Energy Performance
- **Total Energy Use (kWh)**: Overall energy consumption
- **Energy Cost ($/€/etc.)**: Cost based on time-varying energy prices
- **Peak Demand (kW)**: Maximum power requirement
- **Load Shifting**: Ability to move energy consumption to optimal times
- **Carbon Emissions (kg CO2)**: Emissions based on grid carbon intensity

### 2. Thermal Comfort
- **Temperature Violations (K·h)**: Integrated deviation outside comfort bounds
- **Comfort Range Maintenance (%)**: Percentage of time within desired range
- **Predicted Mean Vote (PMV)**: Standardized comfort metric
- **Predicted Percentage Dissatisfied (PPD)**: Expected occupant dissatisfaction percentage

### 3. Control Performance
- **Prediction Accuracy**: How well the model predicts actual temperatures
- **Constraint Satisfaction**: How effectively constraints are maintained
- **Computational Efficiency**: Solution time versus control timestep
- **Robustness**: Performance under uncertainty or disturbances

## Practical Considerations

### MPC Implementation Challenges
- **Model Accuracy**: Performance depends heavily on prediction model quality
- **Computational Requirements**: Solving optimization problems in real-time
- **Forecast Uncertainty**: Weather and price forecasts are never perfect
- **Integration with Existing Systems**: Connecting with legacy building controls

### Model Development
- **Model Identification**: Process of creating thermal models from data
- **Model Validation**: Ensuring models accurately capture building dynamics
- **Model Complexity**: Balancing accuracy with computational efficiency
- **Model Adaptation**: Updating models as building characteristics change

### Solver Selection
- **Interior Point Methods**: Effective for large, sparse optimization problems
- **Active Set Methods**: Can be faster for smaller problems
- **Commercial vs. Open-Source**: Trade-offs in performance, support, and cost

## Advanced Topics

### Stochastic MPC
- Accounts for uncertainty in predictions
- Incorporates probability distributions of weather, occupancy, etc.
- Produces more robust control strategies
- Example:
  ```python
  smpc = epc.high_level_control.StochasticMPC(
      time_step=300,
      weather_scenarios=weather_scenarios,
      confidence_level=0.95,
      risk_measure="CVaR"
  )
  ```

### Economic MPC
- Explicitly optimizes for cost savings
- Incorporates time-varying energy prices
- May utilize thermal mass for load shifting
- Example:
  ```python
  empc = epc.high_level_control.EconomicMPC(
      time_step=300,
      price_forecast=price_forecast,
      objective="min_cost",
      allow_precooling=True
  )
  ```

### Distributed MPC
- Coordinates multiple controllers across zones
- Handles interactions between subsystems
- Suitable for large buildings with multiple zones
- Example:
  ```python
  dmpc = epc.high_level_control.DistributedMPC(
      time_step=300,
      zone_controllers=zone_controllers,
      coordination_method="price_based"
  )
  ```

### Robust MPC
- Ensures constraints are satisfied despite uncertainties
- Provides performance guarantees under worst-case scenarios
- Usually more conservative than standard MPC
- Example:
  ```python
  rmpc = epc.high_level_control.RobustMPC(
      time_step=300,
      uncertainty_bounds=bounds,
      approach="min_max"
  )
  ```

## Exercises

1. **Parameter Sensitivity Analysis**: Experiment with different MPC parameters and observe their effects:
   - Change the prediction horizon (e.g., 24, 48, 72 steps)
   - Modify the slack weight (e.g., 10^4, 10^5, 10^6)
   - Try different comfort bounds (e.g., 19-25°C vs. 20-24°C)
   
   Create a table comparing energy use, cost, and comfort violations for each configuration.

2. **Direct vs. Indirect MPC**: Compare the performance of direct and indirect MPC:
   ```python
   # For direct MPC
   direct_mpc = epc.high_level_control.ModelPredictiveControl(
       # Same parameters as before, but:
       control_type='direct'
   )
   ```
   
   Analyze the differences in control behavior, energy consumption, and comfort maintenance.

3. **Economic Optimization**: Implement a modified MPC that emphasizes cost savings:
   - Add higher weight to price-based optimization
   - Allow pre-heating during low-price periods
   - Compare with standard comfort-focused MPC

4. **Robustness Testing**: Test the controller under unexpected conditions:
   - Introduce model mismatch (e.g., by artificially altering parameters)
   - Add unexpected internal heat gains
   - Simulate sensor noise or failures
   
   Evaluate how the controller responds to these challenges.

```python
# Example exercise implementation: Testing MPC with model mismatch
# First run normal simulation with accurate model
normal_results = eplusman.simulate()

# Then run with artificially introduced model mismatch
# Adjust thermal mass in the model (e.g., multiply by 1.5)
modified_model = model.copy()
modified_model.A = modified_model.A * 1.2  # Increase time constants

mpc_mismatched = epc.high_level_control.ModelPredictiveControl(
    # Same parameters as before, but:
    model=modified_model,
    correct_setpoint=False  # Disable feedback correction
)

eplusman.set_high_level_control(mpc_mismatched)
mismatched_results = eplusman.simulate()

# Compare performance
print("Model Mismatch Effects:")
print(f"Energy consumption increase: {(epc.utils.total_energy(mismatched_results['Heat Power']) - epc.utils.total_energy(normal_results['Heat Power'])) / epc.utils.total_energy(normal_results['Heat Power']) * 100:.1f}%")
print(f"Comfort violation increase: {(epc.utils.temperature_violations(mismatched_results['Indoor Temp'], mismatched_results['lower_setpoint']) - epc.utils.temperature_violations(normal_results['Indoor Temp'], normal_results['lower_setpoint'])):.2f} K·h")
```