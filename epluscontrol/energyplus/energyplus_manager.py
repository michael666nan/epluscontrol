import datetime
import pandas as pd
from pyenergyplus.api import EnergyPlusAPI

from epluscontrol.energyplus.utils.eppy_manager import EppyManager
from epluscontrol.energyplus.utils.file_manager import FileManager
from epluscontrol.energyplus.utils.sensor_manager import SensorManager
from epluscontrol.energyplus.utils.actuator_manager import ActuatorManager
from epluscontrol.energyplus.visualization.plotting import plot_results
from epluscontrol.control.control_managers.standard_control import StandardControl


class EPManager:
    """A class for running EnergyPlus simulations with dynamic control.
    
    This class provides an interface to EnergyPlus for running building energy
    simulations with real-time control and data collection. It supports adding
    sensors, configuring controllers, and visualizing results.
    
    Args:
        idf (str): 
            Path to the EnergyPlus IDF file.
        epw (str): 
            to the EnergyPlus weather file (EPW).
        output_dir (str, optional): 
            Directory for EnergyPlus output files.
            Defaults to "EnergyPlus Output".
        energyplus_dir (str, optional): 
            Path to the EnergyPlus installation directory.
            Used for locating the Energy+.idd file. If None, will try common locations.
            
    Raises:
        ImportError: If eppy package is not installed.
        ValueError: If the IDD file cannot be found.
    """
    
    def __init__(self, idf, epw, energyplus_dir=None, output_dir="ENERGYPLUS_OUTPUT"):
        
        # File handling
        self.file_manager = FileManager(idf, epw, energyplus_dir, output_dir)
        
        # Eppy setup
        self.eppy_manager = EppyManager(energyplus_dir, self.file_manager.get_idf_path())
        
        # Sensor management
        self.sensor_manager = SensorManager()
        
        # Actuator management
        self.actuator_manager = ActuatorManager()
        
        # Initialize EnergyPlus API
        self.api = EnergyPlusAPI()
        self.state = self.api.state_manager.new_state()

        # Control components
        self.low_level_control = None
        self.high_level_control = None
        self.direct_control = None
        self.control_strategy = StandardControl()  # Default strategy
        self.MAX_POWER = 1000  # Default max power (W)

        # Data collection
        self.times = []
        self.setpoints = []
        self.results_df = None
        
    
    def set_run_period(self, start_month=1, start_day=1, end_month=12, end_day=31):
        return self.eppy_manager.set_run_period(start_month, start_day, end_month, end_day)
 
    
    def set_high_level_control(self, controller):
        """Set the high-level controller for setpoint determination.
        
        Args:
            controller: Controller object with a get_setpoint(time) method.
        
        Returns:
            self: For method chaining.
        """
        self.high_level_control = controller
        return self
  
    
    def set_low_level_control(self, controller):
        """Set the low-level controller for tracking setpoints.
        
        Args:
            controller: Controller object with a get_control_output(error) method.
        
        Returns:
            self: For method chaining.
        """
        self.low_level_control = controller
        return self
    
    def set_direct_control(self, controller):
        """Set the direct controller for determining the direct control action.
        
        Args:
            controller: Controller object with a get_control_output method.
        
        Returns:
            self: For method chaining.
        """
        self.direct_control = controller
        return self
 
    
    def set_control_strategy(self, strategy):
        """Set the control strategy to use during simulation.
        
        Args:
            strategy: A ControlStrategy instance defining the control behavior.
        
        Returns:
            self: For method chaining.
        """
        self.control_strategy = strategy
        return self

    
    def set_max_power(self, power):
        """Set the maximum power output for the heating element.
        
        Args:
            power (float): Maximum power in Watts.
        
        Returns:
            self: For method chaining.
        """
        self.MAX_POWER = power
        return self
    
    
    def set_sensors(self, sensor_config):
        """Configure multiple sensors from a dictionary."""
        return self.sensor_manager.set_sensors(sensor_config, self.api, self.state)
    
    def set_actuators(self, actuator_config):
        """Configure multiple actuators from a dictionary."""
        print("This Method is not yet implemented")
    
    def get_handles_callback(self, state):
        """Get Sensor and Actuator Handles."""        
        self.sensor_manager.get_handles(self.api, state)
        self.actuator_manager.get_handles(self.api, state)
        
    
    
    def simulate(self):
        """Run the EnergyPlus simulation with configured sensors and controllers.
        
        Executes the simulation, collects data at each timestep, and returns
        a DataFrame with the results.
        
        Returns:
            pandas.DataFrame: Simulation results with timestamps as index.
            
        Raises:
            ValueError: If required controllers are not set.
        """
        
        # Set up callbacks
        self.api.runtime.callback_begin_new_environment(self.state, self.get_handles_callback)
        self.api.runtime.callback_begin_system_timestep_before_predictor(self.state, self.timestep_callback)        
        
        # Run EnergyPlus
        self.api.runtime.run_energyplus(self.state, ["-w", self.file_manager.get_epw_path(), 
                                                    "-d", self.file_manager.get_output_dir(), 
                                                    self.file_manager.get_idf_path()])

        # Process results
        self.results_df = self._create_data_dict()
        
        # Reset State
        self.api.state_manager.reset_state(self.state)
        
        return self.results_df

    
    def timestep_callback(self, state):
        """Callback that runs at each timestep to collect data and apply control.
        
        This is called by EnergyPlus at each timestep during the simulation.
        The main control logic is delegated to the current control strategy.
        
        Args:
            state: EnergyPlus state object.
        """
        if not self.api.exchange.warmup_flag(state):
            current_time = self._get_current_time()
            
            # Get measured values for all sensors
            self.sensor_manager.get_sensor_values(self.api, state)
            
            # Execute the control strategy
            self.control_strategy.execute_control(state, self, current_time)
            
            # Always record the time
            self.times.append(current_time)

    
    def plot(self, columns=None, figsize=(12, 8), subplot=True, title=None):
        plot_results(self, columns=None, figsize=(12, 8), subplot=True, title=None)

    
    def _create_data_dict(self):
        """Create a pandas DataFrame from collected simulation data.
        
        Returns:
            pandas.DataFrame: Simulation results with timestamps as index.
        """
        data = {"time_stamp": self.times}
        
        # Add data from all sensors
        for name, sensor in self.sensor_manager.sensors.items():
            data[name] = sensor["data"]
        
        # Add setpoint
        data["setpoint"] = self.setpoints
        
        # Add data from high level control
        # for key, value in self.high_level_control.store_data.items():
        #     data[key] = value
            
        df = pd.DataFrame(data)
        df.set_index('time_stamp', inplace=True)
        
        return df

    
    def _get_current_time(self):
        """Get the current simulation time.
        
        Returns:
            datetime.datetime: Current simulation time.
        """
        minute = self.api.exchange.minutes(self.state) - 1
        hour = self.api.exchange.hour(self.state)
        day = self.api.exchange.day_of_month(self.state)
        month = self.api.exchange.month(self.state)
        year = 2023  # Default year
        
        # Create datetime object
        return datetime.datetime(year, month, day, hour, minute)
    
    