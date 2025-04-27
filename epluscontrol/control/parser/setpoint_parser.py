import pandas as pd
from datetime import datetime, timedelta

class SetpointParser:
    """
    Predicts temperature setpoints based on day/night scheduling
    """
    def __init__(self, day_setpoint=21, night_setpoint=19, day_start_hour=6, day_end_hour=20):
        """
        Initializes the Setpoint Predictor
        
        Args:
            day_setpoint: Day temperature setpoint [C].
            night_setpoint: Night temperature setpoint [C].
            day_start_hour: Hour of day from which the day setpoint is used.
            day_end_hour: Hour of day until which the day setpoint is used.
        """
        self.day_setpoint = day_setpoint
        self.night_setpoint = night_setpoint
        self.day_start_hour = day_start_hour
        self.day_end_hour = day_end_hour
    
    def calculate_setpoint(self, timestamp):
        """
        Calculates the setpoint for a given timestamp
        
        Args:
            timestamp: The time in datetime format
            
        Returns:
            The setpoint value
        """
        if timestamp.hour < self.day_start_hour or timestamp.hour >= self.day_end_hour:
            return self.night_setpoint
        else:
            return self.day_setpoint
    
    def get_prediction_horizon(self, timestamp, N):
        """
        Get setpoint predictions for a horizon of N hours starting from timestamp
        
        Parameters:
        -----------
        timestamp : datetime
            The starting timestamp
        N : int
            Number of hours in the prediction horizon (including current hour)
        
        Returns:
        --------
        tuple: (setpoints, timestamps)
            Lists containing the setpoint values and timestamps for the N hours
        """
        # Round to the nearest hour
        hour_timestamp = timestamp.replace(minute=0, second=0, microsecond=0)
        
        # Create list of hour timestamps for the prediction horizon
        timestamps = [hour_timestamp + timedelta(hours=i) for i in range(N)]
        
        # Calculate setpoints for each timestamp
        setpoints = [self.calculate_setpoint(ts) for ts in timestamps]
        
        return setpoints, timestamps
    
    
    def get_values(self, timestamps):
        """
        Get setpoint values for a single timestamp or a list/DatetimeIndex of timestamps.
        
        Args:
            timestamps: A single datetime, or a list/DatetimeIndex of timestamps
        
        Returns:
            float or list: Setpoint value (if single timestamp) or list of values (if multiple timestamps)
        """
        # Handle single timestamp case
        if not hasattr(timestamps, '__iter__') or isinstance(timestamps, str):
            return self.calculate_setpoint(timestamps)
        
        # Handle list/iterable of timestamps
        setpoints = []
        for ts in timestamps:
            setpoint = self.calculate_setpoint(ts)
            setpoints.append(setpoint)
        
        return setpoints
    
    # def get_current_value(self, timestamp):
    #     """
    #     Get setpoint value for a single timestamp. Alias for convenience.
        
    #     Args:
    #         timestamp: A datetime object
        
    #     Returns:
    #         float: Setpoint value for the timestamp
    #     """
    #     return self.get_values_for_timestamps(timestamp)
    
    
    def plot_setpoints(self, start_timestamp, days=2):
        """
        Plot setpoints for a specified number of days
        
        Parameters:
        -----------
        start_timestamp : datetime
            Starting timestamp for the plot
        days : int
            Number of days to plot (default is 2)
        """
        import matplotlib.pyplot as plt
        
        # Calculate how many hours to predict
        hours = days * 24
        
        # Get prediction data
        setpoints, timestamps = self.get_prediction_horizon(start_timestamp, hours)
        
        # Create DataFrame for easier plotting
        df = pd.DataFrame({
            'Timestamp': timestamps,
            'Setpoint': setpoints
        })
        
        # Plot the data
        plt.figure(figsize=(12, 6))
        plt.step(df['Timestamp'], df['Setpoint'], where='post')
        plt.ylabel('Temperature Setpoint (°C)')
        plt.xlabel('Timestamp')
        plt.title('Temperature Setpoint Schedule')
        plt.grid(True)
        plt.ylim(min(self.night_setpoint, self.day_setpoint) - 1, 
                 max(self.night_setpoint, self.day_setpoint) + 1)
        
        # Add horizontal lines to highlight setpoint values
        plt.axhline(y=self.day_setpoint, color='r', linestyle='-', alpha=0.3, 
                   label=f'Day Setpoint ({self.day_setpoint}°C)')
        plt.axhline(y=self.night_setpoint, color='b', linestyle='-', alpha=0.3,
                   label=f'Night Setpoint ({self.night_setpoint}°C)')
        
        # Add vertical lines for day boundaries
        date_range = pd.date_range(start=start_timestamp, periods=days+1, freq='D')
        for date in date_range:
            plt.axvline(x=date, color='gray', linestyle='--', alpha=0.5)
        
        plt.legend()
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()