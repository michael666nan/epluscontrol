import pandas as pd
import matplotlib.pyplot as plt

class GridParser:
    def __init__(self, filepath):
        self.data = pd.read_csv(filepath, sep=';')
        self.data['timestamp'] = pd.to_datetime(self.data['timestamp'])
        self.data['SpotPriceDKKPerkWh'] = self.data['SpotPriceDKKPerkWh'].str.replace(',', '.').astype(float)
        self.data['CO2PerkWh'] = self.data['CO2PerkWh'].str.replace(',', '.').astype(float)
        self.data['hour_timestamp'] = self.data['timestamp'].dt.floor('h')

    
    # def get_values(self, timestamp):
    #     hour_timestamp = timestamp.replace(minute=0, second=0, microsecond=0)
    #     row = self.data[self.data['hour_timestamp'] == hour_timestamp]
    #     if not row.empty:
    #         return row['SpotPriceDKKPerkWh'].iloc[0], row['CO2PerkWh'].iloc[0]
    #     return None, None
    
    def get_values(self, timestamps):
        """Get values for specific timestamp(s).
        
        Parameters
        ----------
        timestamps : datetime or list/DatetimeIndex of datetimes
            The timestamp(s) to look up
                
        Returns
        -------
        If timestamps is a single timestamp:
            tuple: (price, co2) values for the given timestamp,
                   or (None, None) if no data is found
        If timestamps is a list/DatetimeIndex:
            tuple of lists: (price_list, co2_list) where each list contains
                            values for each timestamp, with None for missing data
        """
        # Check if timestamps is a single timestamp
        single_timestamp = not hasattr(timestamps, '__iter__') or isinstance(timestamps, pd.Timestamp)
        
        if single_timestamp:
            # Single timestamp case
            hour_timestamp = timestamps.replace(minute=0, second=0, microsecond=0)
            row = self.data[self.data['hour_timestamp'] == hour_timestamp]
            
            if not row.empty:
                return row['SpotPriceDKKPerkWh'].iloc[0], row['CO2PerkWh'].iloc[0]
            return None, None
        else:
            # Multiple timestamps case
            prices = []
            co2s = []
            
            for ts in timestamps:
                hour_ts = ts.replace(minute=0, second=0, microsecond=0)
                row = self.data[self.data['hour_timestamp'] == hour_ts]
                
                if not row.empty:
                    prices.append(row['SpotPriceDKKPerkWh'].iloc[0])
                    co2s.append(row['CO2PerkWh'].iloc[0])
                else:
                    prices.append(None)
                    co2s.append(None)
            
            return prices, co2s
    
    
    def get_prediction_horizon(self, timestamp, N):
        # Round to the nearest hour
        hour_timestamp = timestamp.replace(minute=0, second=0, microsecond=0)
        
        # Create a list of hour timestamps for the prediction horizon
        timestamps = [hour_timestamp + pd.Timedelta(hours=i) for i in range(N)]
        
        # Get data for each timestprices, co2_values, timestampsamp
        prices = []
        co2_values = []
        
        for ts in timestamps:
            # Find matching row
            row = self.data[self.data['hour_timestamp'] == ts]
            if not row.empty:
                prices.append(row['SpotPriceDKKPerkWh'].iloc[0])
                co2_values.append(row['CO2PerkWh'].iloc[0])
            else:
                # Handle missing data
                prices.append(None)
                co2_values.append(None)
        
        return prices, co2_values, timestamps

    
    # def get_values_for_timestamps(self, timestamps, column_name):
    #     """
    #     Get values from a specific column for multiple timestamps.
        
    #     Args:
    #         timestamps: A list or DatetimeIndex of timestamps
    #         column_name: The name of the column to extract values from
        
    #     Returns:
    #         list: Values from the specified column corresponding to each timestamp
    #     """
    #     values = []
        
    #     for ts in timestamps:
    #         # Round to the nearest hour since your data is hourly
    #         hour_ts = ts.replace(minute=0, second=0, microsecond=0)
            
    #         # Find matching row
    #         row = self.data[self.data['hour_timestamp'] == hour_ts]
            
    #         if not row.empty:
    #             values.append(row[column_name].iloc[0])
    #         else:
    #             values.append(None)
        
    #     return values





    def plot_data(self):
        fig, ax1 = plt.subplots(figsize=(12, 6))

        ax1.plot(self.data['timestamp'], self.data['SpotPriceDKKPerkWh'], color='blue')
        ax1.set_xlabel('Timestamp')
        ax1.set_ylabel('Spot Price (DKK/kWh)', color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.grid(True)

        ax2 = ax1.twinx()
        ax2.plot(self.data['timestamp'], self.data['CO2PerkWh'], color='green')
        ax2.set_ylabel('CO2 per kWh (g/kWh)', color='green')
        ax2.tick_params(axis='y', labelcolor='green')

        plt.title('Spot Price and CO2 Emissions Over Time')
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()
