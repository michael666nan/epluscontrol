from epluscontrol.control.low_level_control.low_level_base import LowLevelController


class BangBangController(LowLevelController):
    """A simple on/off (bang-bang) controller.
    
    This controller outputs 1 (on) when the error is positive and 0 (off)
    when the error is negative or zero. It has no internal state and is
    the simplest form of feedback control.
    
    Args:
        deadband (float, optional): Temperature deadband to prevent frequent
            cycling. For example, with deadband=0.5, the controller will turn
            off when error < 0 and turn on when error > 0.5. Defaults to 0.0.
        time_step (int, optional): Time step between control updates in
            minutes. Defaults to 5.
    """ 
    
    def __init__(self, time_step=5, sensor_name="Indoor Temp", deadband=0.0):
               
        
        super().__init__(time_step, sensor_name)        
        self.deadband = deadband
        self._last_output = 0  # Track last output for hysteresis
    
    def get_control_output(self, error):
        """Calculate the control output based on the error.
        
        Implements a simple hysteresis control with optional deadband.
        
        Args:
            error (float): The difference between setpoint and measured value
                (setpoint - measured_value).
                
        Returns:
            float: 1.0 (on) if error > deadband or if 0 < error <= deadband
                and the last output was 1; 0.0 (off) otherwise.
        """
        
        # Apply deadband with hysteresis
        if error > self.deadband:
            output = 1.0
        elif error < 0:
            output = 0.0
        else:  # In the deadband region, maintain previous state
            output = self._last_output
            
        self._last_output = output
        return output


# Example usage
if __name__ == "__main__":
    # Bang-Bang Controller Example
    print("=== Bang-Bang Controller ===")
    bang_bang = BangBangController(deadband=0.5)
    print(f"Error = -2.0: Output = {bang_bang.get_control_output(-2.0)}")
    print(f"Error = 0.3: Output = {bang_bang.get_control_output(0.3)}")
    print(f"Error = 0.7: Output = {bang_bang.get_control_output(0.7)}")
    
