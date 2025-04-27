from abc import ABC, abstractmethod


class LowLevelController(ABC):
    """Base class for low-level controllers that determine control outputs.
    
    This abstract class defines the interface for all low-level controllers,
    which are responsible for determining control outputs (typically between
    0 and 1) based on the error between setpoint and measured values.
    """
    
    def __init__(self, time_step=5, sensor_name="Indoor Temp"):
        """Initialize the controller with common parameters.
        
        Args:
            time_step (int, optional): Time step between control updates in
                minutes. Defaults to 5.
        """
        self.time_step = time_step
        self.sensor_name = sensor_name
    
    @abstractmethod
    def get_control_output(self, error):
        """Calculate the control output based on the error.
        
        Args:
            error (float): The difference between setpoint and measured value
                (setpoint - measured_value).
                
        Returns:
            float: The control output, typically between 0 and 1.
        """
        pass