class ActuatorManager:
    """
    Manager class for actuator handling.
    - OBS: THIS CLASS CAN ONLY HANDLE A SINGLE SCHEDULE
    
    """    
    def __init__(self):
        """
        Initialize the ActuatorManager.               
        
        Returns:
            self: For method chaining.
        
        """
        self.actuator = None
        
        
    def get_handles(self, api, state):
        """Callback to get variable handles when simulation starts.
        
        This is called by EnergyPlus at the beginning of the simulation.
        
        Args:
            state: EnergyPlus state object.
        """
        # Wait until EnergyPlus is ready
        if not api.exchange.api_data_fully_ready(state):
            return
                
        # Get handle for schedule actuator
        self.actuator = api.exchange.get_actuator_handle(
            state, "Schedule:Constant", "Schedule Value", "HEAT INPUT")

        # Check if handle was successfully retrieved
        if self.actuator == -1:
            print("Warning: Could not get handle for actuator")
            
    def set_actuator_value(self, api, state, value):
        api.exchange.set_actuator_value(state, self.actuator, value)
