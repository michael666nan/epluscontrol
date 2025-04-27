import random

class RandomSetpoint():
    """Controller that randomly switches between min and max setpoints.
    
    This controller is useful for testing system responses or for generating
    excitation signals for system identification.
    
    Parameters
    ----------
    time_step (int, optional): 
        Time step between control updates in minutes. 
        Defaults to 60.
    sensor_name (str):
        Name of the sensor used to measure the controlled variable
    min_setpoint (float, optional):
        Minimum temperature setpoint in °C. 
        Default is 19.
    max_setpoint (float, optional):
        Maximum temperature setpoint in °C. 
        Default is 21.
    random_seed (int, optional):
        Seed for the random number generator. 
        Default is 42.
    change_probability (float, optional):
        Probability of changing the setpoint at each time step (0.0-1.0). 
        Default is 0.25.
    """
    
    def __init__(self, time_step=60, sensor_name=None, min_setpoint=19, max_setpoint=21, random_seed=42, change_probability=0.25):
        
        self.time_step = time_step
        self.sensor_name = sensor_name
        self.min_setpoint = min_setpoint
        self.max_setpoint = max_setpoint
        self.setpoint = max_setpoint
        self.change_probability = change_probability
        
        # Set random seed for reproducibility
        random.seed(random_seed)
        
    def get_setpoint(self, current_time, *args, **kwargs):
        """Calculate the setpoint with random changes at the top of each hour.
        
        At the beginning of each hour, there's a chance (change_probability) that
        the setpoint will change to either min_setpoint or max_setpoint.
        
        Args:
            current_time (datetime): The current simulation time.
            
        Returns:
            float: The temperature setpoint in degrees Celsius.
        """
        # Only consider changing setpoint at the top of each hour
        if current_time.minute % self.time_step == 0:
            if random.random() > (1 - self.change_probability):
                self.setpoint = random.choice([self.min_setpoint, self.max_setpoint])
        
        return self.setpoint