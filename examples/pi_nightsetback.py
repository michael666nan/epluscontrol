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

Prerequisites:
  - EnergyPlus V24-2-0 installed at "C:\EnergyPlusV24-2-0"
  - Project structure with case files at "cases\case_1\"
=======================================================================
"""
import epluscontrol as epc

# =====================================================================
# Step 1: Setting up the simulation
# =====================================================================

# Initialize the simulation manager
eplusman = epc.energyplus.energyplus_manager.EPManager(
    idf="Eplus_project_room.idf",           # energyplus model file
    epw="COPENHAGEN.epw",                   # weather file
    energyplus_dir="C:\EnergyPlusV24-2-0"   # energyplus directory
    )

# Configure simulation period
eplusman.set_run_period(
    start_month=1, 
    start_day=1, 
    end_month=1, 
    end_day=14)

# Set maximum radiator heat output (W)
eplusman.set_max_power(1000)

# =====================================================================
# Step 2: Define Sensors
# =====================================================================
SENSOR_CONFIG = {
    "Indoor Temp": {
        "variable": "ZONE MEAN AIR TEMPERATURE",
        "key": "ZONE 1",
        "unit": "°C"
    },
    "Outdoor Temp": {
        "variable": "SITE OUTDOOR AIR DRYBULB TEMPERATURE",
        "key": "ENVIRONMENT",
        "unit": "°C"
    },
    "Solar Gain": {
        "variable": "Surface Outside Face Incident Solar Radiation Rate per Area",
        "key": "East Wall",
        "unit": "W/m2"
    },
    "Heat Power": {
        "variable": "Electric Equipment Electricity Rate",
        "key": "RADIATOR",
        "unit": "W",
    }
}
# Add Sensors to the Simulation Manager
eplusman.set_sensors(SENSOR_CONFIG)


# =====================================================================
# Step 3: Add Controllers
# =====================================================================
# Low-level control: PI controller
pi = epc.low_level_control.PIController(
    time_step=1, 
    sensor_name="Indoor Temp",      # Must match one of the sensors in SENSOR_CONFIG
    Kp=0.5, 
    tauI=1800.0,
    with_antiwindup=True
    )

# Add the Controller to the Simulation Manager
eplusman.set_low_level_control(pi)

# High-level control: Model Predictive Controller
weather_parser = epc.parser.WeatherParser(filepath="weather.csv")

grid_parser = epc.parser.GridParser(filepath="grid.csv")

low_setpoint_parser = epc.parser.SetpointParser(
    day_setpoint=20,
    night_setpoint=20,
    day_start_hour=6,
    day_end_hour=19)

high_setpoint_parser = epc.parser.SetpointParser(
    day_setpoint=24,
    night_setpoint=24,
    day_start_hour=6,
    day_end_hour=19)

model = epc.StateSpace.load("second_order.npz")

mpc = epc.high_level_control.ModelPredictiveControl(
    time_step=60,
    sensor_name="Indoor Temp",      # Must match one of the sensors in SENSOR_CONFIG
    model=model,
    weather_parser=weather_parser,
    grid_parser=grid_parser,
    low_setpoint_parser=low_setpoint_parser,
    high_setpoint_parser=high_setpoint_parser,
    max_power=eplusman.MAX_POWER,
    horizon=48,
    slack_weight=10e5,
    solver="CLARABEL",
    print_output=True,
    control_type='indirect',
    correct_setpoint=True
    )

const = epc.high_level_control.ConstantSetpoint(
    time_step=60,
    sensor_name="Indoor Temp", 
    setpoint = 27)

night = epc.high_level_control.NightSetback(
    time_step=60,
    sensor_name="Indoor Temp",
    day_setpoint=23,
    night_setpoint=18
    )

rand = epc.high_level_control.RandomSetpoint(
    time_step=60,
    sensor_name="Indoor Temp",
    change_probability=1)


# Add the controller to the Simulation Manager
eplusman.set_high_level_control(mpc)


# =====================================================================
# Step 4: Run the Simulation
# =====================================================================
raw_df = eplusman.simulate()

# =====================================================================
# Step 5: Plot Results
# =====================================================================
eplusman.plot()

# # Add setpoint limits to dataframe
raw_df['lower_setpoint'] = low_setpoint_parser.get_values(raw_df.index)
raw_df['upper_setpoint'] = high_setpoint_parser.get_values(raw_df.index)

# Add electricity prices to dataframe
(raw_df['price'], raw_df['co2']) = grid_parser.get_values(raw_df.index)



# =====================================================================
# Step 7: Plot Results
# =====================================================================
_ = epc.utils.plot_smart_building_data(raw_df)



# Create visualizer
visualizer = epc.utils.MPCVisualizer(mpc)


