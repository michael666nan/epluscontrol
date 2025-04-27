class ConstantSetpoint():
    """Controller that randomly switches between min and max setpoints.
    
    This controller is useful for testing system responses or for generating
    excitation signals for system identification.
    
    Args:        
        time_step (int, optional): 
            Time step between control updates in minutes. 
            Defaults to 60.
        sensor_name (str):
            Name of the sensor used to measure the controlled variable
        setpoint (float):
            Temperature setpoint in °C. 
            Defaults to 20.
    """
    
    def __init__(self, time_step=60, sensor_name=None, setpoint=20):
        self.time_step = time_step
        self.setpoint = setpoint
        self.sensor_name = sensor_name
    
        
    def get_setpoint(self, current_time, *args, **kwargs):
        """Returns the constant setpoint
        
        Args:
            current_time (datetime): The current simulation time.
            
        Returns:
            float: The temperature setpoint in degrees Celsius.
        """
                
        return self.setpoint

