from epluscontrol.control.control_managers.base import ControlStrategy


class StandardControl(ControlStrategy):
    """Standard control strategy using high-level and low-level controllers.
    
    This implements the default control behavior where a high-level controller 
    determines setpoints and a low-level controller tracks those setpoints.
    """
    def __init__(self):
        self.data = {"cost": []}  
    
    
    def execute_control(self, state, simulator, current_time):
        """Execute standard control strategy.
        
        Args:
            state: EnergyPlus state object.
            simulator: The Simulator instance.
            current_time: Current simulation datetime.
            
        Returns:
            dict: Contains the 'setpoint' that was determined.
        """ 
        if hasattr(simulator.high_level_control, "control_type") and simulator.high_level_control.control_type == "direct":
            #Get high-level direct control output            
            setpoint , heat_output = simulator.high_level_control.get_control_output(
                current_time=current_time, 
                simulator=simulator
                )
        else:
            # HIGH-LEVEL CONTROL:
            # Get high-level setpoint
            setpoint, _ = simulator.high_level_control.get_control_output(
                current_time=current_time, 
                simulator=simulator)
            
            # LOW-LEVEL CONTROL
            if current_time.minute % simulator.low_level_control.time_step == 0:
                y_low_level = simulator.sensor_manager.sensors[simulator.low_level_control.sensor_name]["data"][-1]
                error = setpoint - y_low_level
                control_output = simulator.low_level_control.get_control_output(error)
                heat_output = simulator.MAX_POWER * control_output               
        
        simulator.actuator_manager.set_actuator_value(simulator.api, state, heat_output) 
        
        # Store setpoint
        simulator.setpoints.append(setpoint)
        
        return