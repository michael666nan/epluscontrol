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

=======================================================================
"""
import epluscontrol as epc

# =====================================================================
# Step 1: Setting up the simulation
# =====================================================================

# Initialize the simulation manager
# This creates the main interface for controlling the EnergyPlus simulation
# - idf: Path to the EnergyPlus input data file containing building model
# - epw: Path to the EnergyPlus weather file for the location (Copenhagen)
# - energyplus_dir: Directory where EnergyPlus is installed
eplusman = epc.EPManager(
    idf="Eplus_project_room.idf",           # energyplus model file
    epw="COPENHAGEN.epw",                   # weather file
    energyplus_dir="C:\EnergyPlusV24-2-0"   # energyplus directory
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

# =====================================================================
# Step 2: Define Sensors
# =====================================================================
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


# =====================================================================
# Step 3: Add Low-Level Controller
# =====================================================================
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


# =====================================================================
# Step 4: Add High-Level Controller 
# =====================================================================
# The Night Setback controller implements an energy-saving strategy
# It reduces the temperature setpoint during nighttime hours
# Parameters:
# - time_step: How often to update the setpoint (in seconds)
# - sensor_name: Which sensor to monitor (typically just for reference)
# - day_setpoint: Desired temperature during daytime (°C)
# - night_setpoint: Reduced temperature during nighttime (°C)
night = epc.high_level_control.NightSetback(
    time_step=60,               # Update setpoint every minute
    sensor_name="Indoor Temp",  # Reference sensor (not used for control)
    day_setpoint=23,            # 23°C during daytime
    night_setpoint=18,          # 18°C during nighttime (energy saving)
    day_start_hour=6,           # hour when day time starts
    day_end_hour=21             # hour when day time ends
    )


# Add the controller to the Simulation Manager
# This connects the Night Setback controller to the simulation
eplusman.set_high_level_control(night)


# =====================================================================
# Step 5: Run the Simulation
# =====================================================================
# Execute the simulation and store results in a pandas DataFrame
# This runs EnergyPlus with all the configured sensors and controllers
df = eplusman.simulate()

# Print the column names available in the results dataframe
# This shows what data was collected during the simulation
print(df.columns)   # Print columns stored in dataframe


# =====================================================================
# Step 6: Plot Results
# =====================================================================
# Generate standard plots showing simulation results
# This typically includes temperature profiles, setpoints, and power usage
eplusman.plot()  # This creates a default set of plots for visualization


# =====================================================================
# Step 7: Calculate Key Performance Indicators
# =====================================================================
# Calculate the total energy consumption in kilowatt-hours
# This sums up the power usage over time and converts to kWh
tot_energy = epc.utils.total_energy(df['Heat Power'])
print(f"Total energy consumption: {tot_energy} kWh")

# Calculate temperature violations (when room temperature falls below setpoint)
# This measures how well the system maintained the desired temperature
# The result is in Kelvin-hours (K·h), representing the magnitude and duration of violations
below_20 = epc.utils.temperature_violations(df['Indoor Temp'], df['setpoint'], type="lower_bound")
print(f"Kelvin hours below setpoint: {below_20} K·h")

# =====================================================================
# Optional Step 8: Advanced Analysis
# =====================================================================
# Students can extend the analysis with additional code here
# Examples:
# - Compare different control strategies
# - Create custom visualizations
# - Calculate energy costs