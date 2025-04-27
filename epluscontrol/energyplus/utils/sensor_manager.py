class SensorManager:
    """
    Manager class for sensor handling.
    
    """    
    def __init__(self):
        """
        Initialize the SensorManager.        
        Configure multiple sensors from a dictionary.        
        
        Returns:
            self: For method chaining.
        
        """
        self.sensors = {}
        self.actuators = None


    def set_sensors(self, sensor_config, api, state):
        """Configure multiple sensors from a dictionary.
        
        Args:
            sensor_config (dict): Dictionary with sensor configurations.
                Each sensor should have at minimum 'variable' and 'key' fields.
                Additional fields like 'unit' and 'agg_method' are also stored.
        
        Returns:
            dictionary of sensors
        
        Example:
            >>> simulator.set_sensors({
            ...     "indoor_temp": {
            ...         "variable": "ZONE MEAN AIR TEMPERATURE",
            ...         "key": "ZONE 1",
            ...         "unit": "°C",
            ...         "agg_method": "first"
            ...     }
            ... })
        """
        for name, config in sensor_config.items():
            # Create a copy of the configuration to avoid modifying the original
            sensor_info = config.copy()
            
            # Ensure required fields are present
            if 'variable' not in sensor_info or 'key' not in sensor_info:
                print(f"Warning: Sensor '{name}' is missing required fields (variable, key). Skipping.")
                continue
            
            # Add handle and data fields that will be populated during simulation
            sensor_info['handle'] = None
            sensor_info['data'] = []
            
            # Store the complete sensor configuration
            self.sensors[name] = sensor_info
            
            # Request this variable from EnergyPlus
            api.exchange.request_variable(state, sensor_info['variable'], sensor_info['key'])
            
            unit_str = f", unit: {sensor_info.get('unit', 'not specified')}" if 'unit' in sensor_info else ""
            print(f"Added sensor: {name} ({sensor_info['variable']}, {sensor_info['key']}{unit_str})")
        
    
    
    def get_handles(self, api, state):
        """Callback to get variable handles when simulation starts.
        
        This is called by EnergyPlus at the beginning of the simulation.
        
        Args:
            state: EnergyPlus state object.
        """
        # Wait until EnergyPlus is ready
        if not api.exchange.api_data_fully_ready(state):
            return
        
        # Get handles for all configured sensors
        for name, sensor in self.sensors.items():
            sensor["handle"] = api.exchange.get_variable_handle(
                state, sensor["variable"], sensor["key"])
            
            # Check if handle was successfully retrieved
            if sensor["handle"] == -1:
                print(f"Warning: Could not get handle for {name} ({sensor['variable']}, {sensor['key']})")
 
    
    def get_sensor_values(self, api, state):
        # Get measured values for all sensors
        for name, sensor in self.sensors.items():
            if sensor["handle"] != -1:  # Only get value if handle is valid
                value = api.exchange.get_variable_value(state, sensor["handle"])
                sensor["data"].append(value)
            else:
                # Handle missing data by appending None or a default value
                print("Missing Values")
                sensor["data"].append(None)