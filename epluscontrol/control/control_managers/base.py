from abc import ABC, abstractmethod

class ControlStrategy(ABC):
    """Base abstract class for EnergyPlus control strategies.
    
    This class defines the interface for timestep control strategies
    that can be used with the Simulator class.
    """
    
    @abstractmethod
    def execute_control(self, state, simulator, current_time):
        """Execute control actions for the current timestep.
        
        Args:
            state: EnergyPlus state object.
            simulator: The Simulator instance.
            current_time: Current simulation datetime.
            
        Returns:
            dict: Additional data to be stored for this timestep (optional).
        """
        pass