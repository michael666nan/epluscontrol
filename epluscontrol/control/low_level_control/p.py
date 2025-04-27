from epluscontrol.control.low_level_control.low_level_base import LowLevelController

class PController(LowLevelController):
    """A simple Proportional controller.
    
    This controller outputs a control signal proportional to the error,
    with optional scaling and offset.
    
    Args:
        Kp (float, optional): Proportional gain. Defaults to 1.0.
        output_min (float, optional): Minimum output value. Defaults to 0.0.
        output_max (float, optional): Maximum output value. Defaults to 1.0.
        offset (float, optional): Offset added to the proportional term.
            Can be used to set a non-zero output at zero error. Defaults to 0.0.
        time_step (int, optional): Time step between control updates in
            minutes. Defaults to 5.
        """   
    
    def __init__(self, time_step=5, sensor_name="Indoor Temp", Kp=1.0, output_min=0.0, output_max=1.0, offset=0.0):

        super().__init__(time_step, sensor_name)
        self.Kp = Kp
        self.output_min = output_min
        self.output_max = output_max
        self.offset = offset
    
    def get_control_output(self, error):
        """Calculate the control output based on the error.
        
        Args:
            error (float): The difference between setpoint and measured value
                (setpoint - measured_value).
                
        Returns:
            float: Control output, constrained between output_min and output_max.
        """
        # Calculate proportional term with offset
        output = self.Kp * error + self.offset
        
        # Constrain output between min and max
        output = min(max(output, self.output_min), self.output_max)
        
        return output
