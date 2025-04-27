import pandas as pd
import matplotlib.pyplot as plt

class WeatherParser:
    """Parser for weather data from CSV files.
    
    This class handles loading, processing, and retrieving weather data,
    particularly outdoor temperature and solar radiation.
    
    Parameters
    ----------
    filepath : str
        Path to the CSV file containing weather data. The CSV should include
        'Timestamp', 'outdoor' (temperature), and 'solar' (radiation) columns.
    """
    
    def __init__(self, filepath):
        # Read the CSV file
        self.data = pd.read_csv(filepath)
        
        # Convert timestamp to datetime
        self.data['Timestamp'] = pd.to_datetime(self.data['Timestamp'])
        
        # Add hour_timestamp column for easier lookup
        self.data['hour_timestamp'] = self.data['Timestamp'].dt.floor('h')
    
    def get_values(self, timestamps):
        """Get weather data for specific timestamp(s).
        
        Parameters
        ----------
        timestamps : datetime or list/DatetimeIndex of datetimes
            The timestamp(s) to look up
                
        Returns
        -------
        If timestamps is a single timestamp:
            tuple: (outdoor_temp, solar_radiation) values for the given timestamp,
                   or (None, None) if no data is found
        If timestamps is a list/DatetimeIndex:
            tuple of lists: (outdoor_temps, solar_radiations) where each list contains
                            values for each timestamp, with None for missing data
        """
        # Check if timestamps is a single timestamp
        single_timestamp = not hasattr(timestamps, '__iter__') or isinstance(timestamps, pd.Timestamp)
        
        if single_timestamp:
            # Single timestamp case
            hour_timestamp = timestamps.replace(minute=0, second=0, microsecond=0)
            row = self.data[self.data['hour_timestamp'] == hour_timestamp]
            
            if not row.empty:
                return row['outdoor'].iloc[0], row['solar'].iloc[0]
            return None, None
        else:
            # Multiple timestamps case
            outdoor_temps = []
            solar_radiations = []
            
            for ts in timestamps:
                hour_ts = ts.replace(minute=0, second=0, microsecond=0)
                row = self.data[self.data['hour_timestamp'] == hour_ts]
                
                if not row.empty:
                    outdoor_temps.append(row['outdoor'].iloc[0])
                    solar_radiations.append(row['solar'].iloc[0])
                else:
                    outdoor_temps.append(None)
                    solar_radiations.append(None)
            
            return outdoor_temps, solar_radiations
    
    
    def get_prediction_horizon(self, timestamp, N):
        """Get weather data for a prediction horizon of N hours.
        
        Parameters
        ----------
        timestamp : datetime
            The starting timestamp
        N : int
            Number of hours in the prediction horizon (including current hour)
        
        Returns
        -------
        tuple
            Three lists containing (outdoor_temps, solar_values, timestamps)
            for the requested prediction horizon
        """
        # Round to the nearest hour
        hour_timestamp = timestamp.replace(minute=0, second=0, microsecond=0)
        
        # Create list of hour timestamps for the prediction horizon
        timestamps = [hour_timestamp + pd.Timedelta(hours=i) for i in range(N)]
        
        # Get data for each timestamp
        outdoor_temps = []
        solar_values = []
        
        for ts in timestamps:
            # Find matching row
            row = self.data[self.data['hour_timestamp'] == ts]
            if not row.empty:
                outdoor_temps.append(row['outdoor'].iloc[0])
                solar_values.append(row['solar'].iloc[0])
            else:
                # Handle missing data
                outdoor_temps.append(None)
                solar_values.append(None)
        
        return outdoor_temps, solar_values, timestamps
    
    def plot_data(self, days=7):
        """Plot weather data for visualization.
        
        Creates a two-panel plot showing outdoor temperature and solar radiation
        for the specified number of days from the start of the dataset.
        
        Parameters
        ----------
        days : int, optional
            Number of days to plot. Default is 7.
        
        Returns
        -------
        None
            Displays the plot using matplotlib
        """
        # Get the first days worth of data
        end_date = self.data['Timestamp'].min() + pd.Timedelta(days=days)
        plot_data = self.data[self.data['Timestamp'] <= end_date].copy()
        
        # Create the figure
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        # Plot outdoor temperature
        ax1.plot(plot_data['Timestamp'], plot_data['outdoor'], color='blue')
        ax1.set_ylabel('Temperature (°C)', color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.grid(True)
        ax1.set_title('Outdoor Temperature')
        
        # Plot solar radiation
        ax2.plot(plot_data['Timestamp'], plot_data['solar'], color='orange')
        ax2.set_ylabel('Solar Radiation (W/m²)', color='orange')
        ax2.tick_params(axis='y', labelcolor='orange')
        ax2.grid(True)
        ax2.set_title('Solar Radiation')
        
        # Add overall title and format x-axis
        plt.suptitle('Weather Data')
        plt.xlabel('Timestamp')
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()