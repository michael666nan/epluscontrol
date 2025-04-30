"""
=======================================================================
Example: EnergyPlus Simulation with Model Predictive Control
=======================================================================

Description:
This tutorial demonstrates how to implement Model Predictive Control (MPC)
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

# =====================================================================
# Step 1: Setting up the simulation environment
# =====================================================================

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

# =====================================================================
# Step 2: Configure Sensing Points (Measurements)
# =====================================================================
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


# =====================================================================
# Step 3: Configure Low-Level Controller (For Indirect MPC)
# =====================================================================
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


# =====================================================================
# Step 4: Configure Model Predictive Controller (High-Level)
# =====================================================================
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
    control_type='direct',        # Use indirect MPC (setpoint control)
    correct_setpoint=True           # Apply feedback correction for model mismatch
    )


rand = epc.high_level_control.RandomSetpoint(
    sensor_name="Indoor Temp",
    change_probability=1
    )

const = epc.high_level_control.ConstantSetpoint()

# Register the MPC controller with the simulation manager
# This establishes the MPC as the high-level decision maker
eplusman.set_high_level_control(rand)


# =====================================================================
# Step 5: Execute the Simulation
# =====================================================================
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


# =====================================================================
# Step 6: Visualize Simulation Results
# =====================================================================
# Generate standard visualization of simulation outcomes
# This includes temperature profiles, setpoints, and heating power
eplusman.plot()  # Default plots from the simulation manager


# =====================================================================
# Step 7: Evaluate Performance Metrics (KPIs)
# =====================================================================
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


# =====================================================================
# Step 8: Advanced Visualization
# =====================================================================
# Generate comprehensive smart building visualization
# This shows the interrelationships between various system variables
_ = epc.utils.plot_smart_building_data(df)

# Create interactive MPC visualization tool
# This displays historical data and predictions at each MPC timestep
#visualizer = epc.utils.MPCVisualizer(mpc)
# To launch the interactive visualizer, call:
# visualizer.run()