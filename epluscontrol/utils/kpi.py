from typing import List


def total_energy(energy: List[float], sample_time: int = 60) -> float:
    """
    Calculate the total energy consumption in kilowatt-hours (kWh).

    Args:
        energy (List[float]): A list of power values in watts (W).
        sample_time (int, optional): The time interval between samples in seconds. Defaults to 60.

    Returns:
        float: The total energy consumption in kilowatt-hours (kWh).
    """
    return sum(energy) * sample_time / 3600000  # Convert J to kWh



def total_energy_cost(energy: List[float], price: List[float], sample_time: int = 60) -> float:
    """
    Calculate the total energy cost based on power consumption and price per kWh.

    Args:
        energy (List[float]): A list of power values in watts (W).
        price (List[float]): A list of price values in DKK per kWh.
        sample_time (int, optional): The time interval between samples in seconds. Defaults to 60.

    Returns:
        float: The total energy cost in DKK.
    """
    if len(energy) != len(price):
        raise ValueError("Energy and price lists must have the same length.")
    
    total_cost = sum(e * p for e, p in zip(energy, price)) * (sample_time / 3600000)
    
    return total_cost


from typing import List, Union
import pandas as pd
import numpy as np

def temperature_violations(temperature: Union[List[float], pd.Series], 
                          constraint: Union[float, List[float], pd.Series], 
                          type: str = "lower_bound", 
                          sample_time: int = 60) -> float:
    """
    Calculate temperature violations in Kelvin-hours.
    Args:
        temperature (Union[List[float], pd.Series]): A list or Series of recorded temperature values (°C or K).
        constraint (Union[float, List[float], pd.Series]): Either a single temperature constraint value 
                                                         or a list/Series of temperature constraints.
        type (str, optional): "lower_bound" to sum violations below the constraint, 
                              "upper_bound" to sum violations above it. Defaults to "lower_bound".
        sample_time (int, optional): The time interval between samples in seconds. Defaults to 60.
    Returns:
        float: The total temperature violation in Kelvin-hours (K·h).
    """
    # Convert inputs to numpy arrays for consistent handling
    temp_array = np.array(temperature)
    
    # Handle single constraint value
    if isinstance(constraint, (int, float)):
        const_array = np.full_like(temp_array, constraint)
    else:
        const_array = np.array(constraint)
    
    # Check lengths
    if len(temp_array) != len(const_array):
        raise ValueError("Temperature and constraint arrays must have the same length.")
    
    # Compute violations
    if type == "lower_bound":
        # For lower bound, violation occurs when temperature is below constraint
        violations = np.maximum(0, const_array - temp_array)
    elif type == "upper_bound":
        # For upper bound, violation occurs when temperature is above constraint
        violations = np.maximum(0, temp_array - const_array)
    else:
        raise ValueError("Type must be 'lower_bound' or 'upper_bound'.")
    
    # Sum violations and scale to Kelvin-hours
    total_violation = np.sum(violations)
    return total_violation * (sample_time / 3600)