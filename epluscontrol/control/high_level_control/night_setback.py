class NightSetback():
    """Controller that implements a day/night temperature setback strategy.
    
    This controller sets different temperature setpoints for day and night periods,
    which is a common energy-saving strategy in building control.
    
    Args:
        time_step (int, optional): 
            Time step between control updates in minutes. 
            Defaults to 60.
        sensor_name (str):
            Name of the sensor used to measure the controlled variable
        day_setpoint (float): 
            Daytime temperature setpoint in °C. 
            Defaults to 21.
        night_setpoint (float): 
            Nighttime temperature setpoint in °C. 
            Defaults to 19.
        day_start_hour (int): 
            Hour when day period begins (0-23). 
            Defaults to 6.
        day_end_hour (int): 
            Hour when night period begins (0-23). 
            Defaults to 20.
        time_step (int, optional): 
            Time step between control updates in minutes. 
            Defaults to 60.
    """  
    
    def __init__(self, time_step=60, sensor_name=None, day_setpoint=21, night_setpoint=19, day_start_hour=6, day_end_hour=20):
    
        self.time_step = time_step
        self.sensor_name = sensor_name
        self.day_setpoint = day_setpoint
        self.night_setpoint = night_setpoint
        self.day_start_hour = day_start_hour
        self.day_end_hour = day_end_hour
    
    def get_control_output(self, current_time, *args, **kwargs):
        """Calculate the setpoint based on the time of day.
        
        During day hours (day_start_hour to day_end_hour), the day setpoint is used.
        During night hours, the night setpoint is used.
        
        Args:
            current_time (datetime): The current simulation time.
            
        Returns:
            float: The temperature setpoint in degrees Celsius.
        """
        if current_time.hour < self.day_start_hour or current_time.hour >= self.day_end_hour:
            setpoint = self.night_setpoint
        else:
            setpoint = self.day_setpoint
        
        return setpoint, None