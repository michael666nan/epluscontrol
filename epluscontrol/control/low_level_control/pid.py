from epluscontrol.control.low_level_control.low_level_base import LowLevelController

class PIDController(LowLevelController):
    """A Proportional-Integral-Derivative (PID) controller with anti-windup.
    
    This controller extends the PI controller by adding a derivative term,
    which can improve response to rapid changes but may amplify noise.

     Args:
         Kp (float, optional): Proportional gain. Defaults to 1.0.
         tauI (float, optional): Integral time constant in seconds.
             Defaults to 1800.0 (30 minutes).
         tauD (float, optional): Derivative time constant in seconds.
             Defaults to 60.0 (1 minute).
         with_antiwindup (bool, optional): Whether to use anti-windup
             protection. Defaults to True.
         initial_ierror (float, optional): Initial value for the integrated
             error. Defaults to 0.0.
         time_step (int, optional): Time step between control updates in
             minutes. Defaults to 5.
         derivative_filter (float, optional): Filtering coefficient for the
             derivative term (0-1). Lower values provide more filtering.
             Defaults to 0.1.
         time_step (int, optional): Time step between control updates in
             minutes. Defaults to 5.
             """        
    
    def __init__(self, time_step=5, sensor_name="Indoor Temp", Kp=1.0, tauI=1800.0, tauD=60.0, 
                 with_antiwindup=True, initial_ierror=0.0,
                 derivative_filter=0.1):
    
        super().__init__(time_step, sensor_name)
        self.Kp = Kp
        self.tauI = tauI
        self.tauD = tauD
        self.with_antiwindup = with_antiwindup
        self.ierror = initial_ierror
        self.time_step = time_step
        self.derivative_filter = derivative_filter
        
        # Initialize derivative calculation
        self.prev_error = 0.0
        self.filtered_derror = 0.0
    
    def get_control_output(self, error):
        """Calculate the control output and update the internal state.
        
        Implements a PID controller with anti-windup protection and
        derivative filtering.
        
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
        
        # Calculate raw derivative (error rate of change)
        derror = (error - self.prev_error) / delta_t
        
        # Apply filtering to derivative term to reduce noise sensitivity
        self.filtered_derror = self.filtered_derror * (1 - self.derivative_filter) + derror * self.derivative_filter
        
        # Calculate PID terms
        P = self.Kp * error
        I = self.Kp / self.tauI * new_ierror
        D = self.Kp * self.tauD * self.filtered_derror
        
        # Calculate raw control output
        raw_output = P + I + D
        
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
        self.prev_error = error
        
        return control_output


# Example usage
if __name__ == "__main__":    
    # PID Controller Example
    print("\n=== PID Controller ===")
    pid = PIDController(Kp=0.5, tauI=600, tauD=30)
    print(f"Step 1 - Error = 2.0: Output = {pid.get_control_output(2.0)}")
    print(f"Step 2 - Error = 1.5: Output = {pid.get_control_output(1.5)}")
    print(f"Step 3 - Error = 1.0: Output = {pid.get_control_output(1.0)}")