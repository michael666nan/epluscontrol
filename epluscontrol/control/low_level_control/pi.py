from epluscontrol.control.low_level_control.low_level_base import LowLevelController

class PIController(LowLevelController):
    """A Proportional-Integral (PI) controller with optional anti-windup.
    
    This controller calculates control outputs based on proportional and
    integral components of the error. It includes anti-windup protection
    to prevent integral windup when the output saturates.

    Args:
        Kp (float, optional): Proportional gain. Defaults to 1.0.
        tauI (float, optional): Integral time constant in seconds.
            Defaults to 1800.0 (30 minutes).
        with_antiwindup (bool, optional): Whether to use anti-windup
            protection. Defaults to True.
        initial_ierror (float, optional): Initial value for the integrated
            error. Defaults to 0.0.
        time_step (int, optional): Time step between control updates in
            minutes. Defaults to 5.
    """        
    
    def __init__(self, time_step=5, sensor_name="Indoor Temp", Kp=1.0, tauI=1800.0, with_antiwindup=True, 
                 initial_ierror=0.0):
        
        super().__init__(time_step, sensor_name)
        self.Kp = Kp
        self.tauI = tauI
        self.with_antiwindup = with_antiwindup
        self.ierror = initial_ierror
        self.time_step = time_step
    
    def get_control_output(self, error):
        """Calculate the control output and update the internal state.
        
        Implements a PI controller with optional anti-windup protection.
        
        Args:
            error (float): The difference between setpoint and measured value
                (setpoint - measured_value).
                
        Returns:
            float: Control output, constrained between 0 and 1.
        """
        # Calculate the time step in seconds
        delta_t = self.time_step * 60
        
        # Calculate new integrated error (before anti-windup)
        new_ierror = self.ierror + error * delta_t
        
        # Calculate proportional and integral terms
        P = self.Kp * error
        I = self.Kp / self.tauI * new_ierror
        
        # Calculate raw control output
        raw_output = P + I
        
        # Constrain output between 0 and 1
        control_output = min(max(raw_output, 0.0), 1.0)
        
        # Apply anti-windup if enabled
        if self.with_antiwindup:
            if control_output >= 1.0 and raw_output > 1.0:
                # Output is saturated high and would continue to increase
                new_ierror = self.ierror
            elif control_output <= 0.0 and raw_output < 0.0:
                # Output is saturated low and would continue to decrease
                new_ierror = self.ierror
        
        # Update internal state
        self.ierror = new_ierror
        
        return control_output
