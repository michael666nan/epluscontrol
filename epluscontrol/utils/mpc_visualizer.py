"""
MPC Visualization Tool with Hourly Steps

This visualizer shows MPC data at hourly intervals that match the controller timesteps,
even when historical data is collected more frequently.
"""

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

class MPCVisualizer:
    """MPC visualization tool for hourly controller steps."""
    
    def __init__(self, mpc, title="MPC Visualization"):
        """Initialize the visualizer with MPC data.
        
        Args:
            mpc: 
                MPC controller instance with history and prediction_horizons
            title: 
                Title for the visualization window
        """
        # Store references to MPC data
        self.mpc = mpc
        self.history = mpc.history
        self.prediction_horizons = mpc.prediction_horizons
        self.controller_timestep_minutes = mpc.time_step
        
        # Extract only the controller timesteps from history data
        self._filter_controller_timesteps()
        
        # Determine fixed y-axis limits
        self._calculate_axis_limits()
        
        # Setup Tkinter
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("1000x800")
        
        # Create control frame
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # Add timestamp display
        self.timestamp_var = tk.StringVar(value="")
        ttk.Label(control_frame, text="Current Time:").pack(side=tk.LEFT)
        ttk.Label(control_frame, textvariable=self.timestamp_var, 
                 font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=5)
        
        # Add slider for timestep navigation
        slider_frame = ttk.Frame(self.root)
        slider_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        ttk.Label(slider_frame, text="Timestep:").pack(side=tk.LEFT)
        self.slider_var = tk.IntVar(value=0)
        self.time_slider = ttk.Scale(slider_frame, from_=0, to=1, 
                                     orient=tk.HORIZONTAL,
                                     variable=self.slider_var,
                                     command=self._slider_changed)
        self.time_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Navigation buttons
        self.prev_btn = ttk.Button(control_frame, text="Previous", command=self._prev_step)
        self.prev_btn.pack(side=tk.LEFT, padx=10)
        
        self.next_btn = ttk.Button(control_frame, text="Next", command=self._next_step)
        self.next_btn.pack(side=tk.LEFT, padx=10)
        
        # Step counter
        self.step_label = ttk.Label(control_frame, text="0/0")
        self.step_label.pack(side=tk.RIGHT, padx=5)
        
        # Create the figure with subplots
        fig_frame = ttk.Frame(self.root)
        fig_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.fig = Figure(figsize=(10, 8))
        
        # Add a text box for current timestep at the top of the figure
        self.fig.suptitle("", fontsize=12)
        self.timestamp_text = self.fig.text(0.5, 0.98, "", 
                                           ha='center', va='top',
                                           bbox=dict(boxstyle="round,pad=0.3", 
                                                  fc="white", ec="black", alpha=0.8),
                                           fontsize=10)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=fig_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Add toolbar
        toolbar = NavigationToolbar2Tk(self.canvas, fig_frame)
        toolbar.update()
        
        # Create subplots - adding a fourth subplot for outdoor temp and solar radiation
        self.ax_temp = self.fig.add_subplot(4, 1, 1)
        self.ax_power = self.fig.add_subplot(4, 1, 2, sharex=self.ax_temp)
        self.ax_price = self.fig.add_subplot(4, 1, 3, sharex=self.ax_temp)
        self.ax_outdoor = self.fig.add_subplot(4, 1, 4, sharex=self.ax_temp)
        
        # Create twin axis for solar radiation
        self.ax_solar = self.ax_outdoor.twinx()
        
        # Create info text area
        info_frame = ttk.LabelFrame(self.root, text="Controller Information")
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        self.info_text = tk.Text(info_frame, height=3, wrap=tk.WORD)
        self.info_text.pack(fill=tk.X, padx=5, pady=5)
        
        # Initialize the step index and max index
        self.current_index = 0
        self.max_index = len(self.controller_timestamps) - 1 if self.controller_timestamps else 0
        
        # Update slider range
        if self.max_index > 0:
            self.time_slider.configure(to=self.max_index)
        
        # Update the display with first step
        if self.max_index >= 0:
            self._update_display(0)
    
    def _filter_controller_timesteps(self):
        """Extract only the timestamps that match controller steps."""
        all_timestamps = self.history['timestamp']
        
        # Convert to datetime objects if needed
        timestamps = []
        for ts in all_timestamps:
            if isinstance(ts, str):
                try:
                    dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                    timestamps.append(dt)
                except:
                    timestamps.append(ts)
            else:
                timestamps.append(ts)
        
        # Filter timestamps to keep only those at controller timesteps
        self.controller_timestamps = []
        self.controller_indices = []
        
        for i, ts in enumerate(timestamps):
            if isinstance(ts, datetime):
                # Check if minutes and seconds are zero or match the controller timestep
                if (ts.minute % self.controller_timestep_minutes == 0 and ts.second == 0):
                    self.controller_timestamps.append(ts)
                    self.controller_indices.append(i)
        
        # Sort prediction horizon keys
        self.prediction_keys = sorted(list(self.prediction_horizons.keys()))
    
    def _calculate_axis_limits(self):
        """Calculate fixed y-axis limits for all plots."""
        # Temperature axis limits
        all_temps = self.history['measured_temperature']
        
        # Also include temperatures from prediction horizons
        for key, pred_data in self.prediction_horizons.items():
            if 'predicted_temperatures' in pred_data:
                all_temps = all_temps + list(pred_data['predicted_temperatures'])
        
        # Get comfort bounds if available
        min_bound = 19  # Default minimum bound
        max_bound = 23  # Default maximum bound
        
        for key, pred_data in self.prediction_horizons.items():
            if 'lower_bounds' in pred_data and len(pred_data['lower_bounds']) > 0:
                min_bound = min(min_bound, min(pred_data['lower_bounds']))
            if 'upper_bounds' in pred_data and len(pred_data['upper_bounds']) > 0:
                max_bound = max(max_bound, max(pred_data['upper_bounds']))
        
        # Calculate temperature limits with padding
        temp_min = min(min(all_temps), min_bound) - 1
        temp_max = max(max(all_temps), max_bound) + 1
        self.temp_limits = (temp_min, temp_max)
        
        # Heating power axis limits
        all_power = self.history['applied_heating_power']
        
        # Include predicted heating powers
        for key, pred_data in self.prediction_horizons.items():
            if 'heating_power' in pred_data:
                all_power = all_power + list(pred_data['heating_power'])
        
        # Calculate power limits with padding
        power_min = max(0, min(all_power) - 100)  # Don't go below 0
        power_max = max(all_power) + 100
        self.power_limits = (power_min, power_max)
        
        # Energy price axis limits (if available)
        if 'energy_price' in self.history:
            all_prices = self.history['energy_price']
            
            # Include predicted prices
            for key, pred_data in self.prediction_horizons.items():
                if 'energy_price' in pred_data:
                    all_prices = all_prices + list(pred_data['energy_price'])
            
            # Calculate price limits with padding
            price_min = max(0, min(all_prices) - 0.01)  # Don't go below 0
            price_max = max(all_prices) + 0.01
            self.price_limits = (price_min, price_max)
        else:
            self.price_limits = (0, 1)  # Default if no price data
            
        # Outdoor temperature axis limits (if available)
        if 'outdoor_temperature' in self.history:
            all_outdoor_temps = self.history['outdoor_temperature']
            
            # Include predicted outdoor temperatures if available
            for key, pred_data in self.prediction_horizons.items():
                if 'outdoor_temperature' in pred_data:
                    all_outdoor_temps = all_outdoor_temps + list(pred_data['outdoor_temperature'])
            
            # Calculate outdoor temperature limits with padding
            outdoor_temp_min = min(all_outdoor_temps) - 2
            outdoor_temp_max = max(all_outdoor_temps) + 2
            self.outdoor_temp_limits = (outdoor_temp_min, outdoor_temp_max)
        else:
            # Default limits if no outdoor temperature data
            self.outdoor_temp_limits = (0, 30)
            
        # Solar radiation axis limits (if available)
        if 'solar_radiation' in self.history:
            all_solar = self.history['solar_radiation']
            
            # Include predicted solar radiation if available
            for key, pred_data in self.prediction_horizons.items():
                if 'solar_radiation' in pred_data:
                    all_solar = all_solar + list(pred_data['solar_radiation'])
            
            # Calculate solar radiation limits with padding
            solar_min = max(0, min(all_solar))  # Don't go below 0
            solar_max = max(all_solar) * 1.1  # Add 10% padding
            self.solar_limits = (solar_min, solar_max)
        else:
            # Default limits if no solar radiation data
            self.solar_limits = (0, 1000)
    
    def _prev_step(self):
        """Go to previous step."""
        if self.current_index > 0:
            self.current_index -= 1
            self.slider_var.set(self.current_index)
            self._update_display(self.current_index)
    
    def _next_step(self):
        """Go to next step."""
        if self.current_index < self.max_index:
            self.current_index += 1
            self.slider_var.set(self.current_index)
            self._update_display(self.current_index)
            
    def _slider_changed(self, value):
        """Handle slider value changes."""
        # Convert string value to int and ensure it's within bounds
        index = min(max(int(float(value)), 0), self.max_index)
        if index != self.current_index:
            self.current_index = index
            self._update_display(index)
    
    def _update_display(self, index):
        """Update the display for the given controller timestep index."""
        if index < 0 or index > self.max_index:
            return
            
        # Get current controller timestamp
        current_time = self.controller_timestamps[index]
        
        # Get the original history index for this controller step
        history_index = self.controller_indices[index]
        
        # Format timestamp for display
        if isinstance(current_time, datetime):
            time_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
        else:
            time_str = str(current_time)
        
        self.timestamp_var.set(time_str)
        self.step_label.config(text=f"{index+1}/{self.max_index+1}")
        
        # Get history data up to current controller step
        history_indices = self.controller_indices[:index+1]
        
        history_timestamps = [self.history['timestamp'][i] for i in history_indices]
        actual_temps = [self.history['measured_temperature'][i] for i in history_indices]
        setpoints = [self.history['applied_setpoint'][i] for i in history_indices]
        heating_powers = [self.history['applied_heating_power'][i] for i in history_indices]
        
        # Get energy prices if available
        energy_prices = None
        if 'energy_price' in self.history:
            energy_prices = [self.history['energy_price'][i] for i in history_indices]
            
        # Get outdoor temperature and solar radiation if available
        outdoor_temps = None
        if 'outdoor_temperature' in self.history:
            outdoor_temps = [self.history['outdoor_temperature'][i] for i in history_indices]
            
        solar_radiation = None
        if 'solar_radiation' in self.history:
            solar_radiation = [self.history['solar_radiation'][i] for i in history_indices]
        
        # Clear previous plots
        self.ax_temp.clear()
        self.ax_power.clear()
        self.ax_price.clear()
        self.ax_outdoor.clear()
        self.ax_solar.clear()
        
        # Plot historical data
        self.ax_temp.plot(history_timestamps, actual_temps, 'b-',
                          label='Measured Temperature')
        self.ax_temp.step(history_timestamps, setpoints, 'r-', where='post', 
                          label='Applied Setpoint')
        self.ax_power.step(history_timestamps, heating_powers, 'g-', where='pre', 
                           label='Applied Power')
        
        if energy_prices:
            self.ax_price.step(history_timestamps, energy_prices, 'r-', where='pre', 
                              marker='o', markersize=5, label='Energy Price')
            
        # Plot outdoor temperature and solar radiation
        if outdoor_temps:
            self.ax_outdoor.plot(history_timestamps, outdoor_temps, 'b-',
                                label='Outdoor Temperature')
            
        if solar_radiation:
            self.ax_solar.plot(history_timestamps, solar_radiation, 'y-', 
                              label='Solar Radiation')
        
        # Find the matching prediction for this timestamp
        prediction_key = None
        if time_str in self.prediction_horizons:
            prediction_key = time_str
        
        # If no exact match, try to format the timestamp
        if not prediction_key and isinstance(current_time, datetime):
            alt_format = current_time.strftime("%Y-%m-%d %H:%M:%S")
            if alt_format in self.prediction_horizons:
                prediction_key = alt_format
        
        # Plot prediction data if available
        has_prediction = False
        if prediction_key and prediction_key in self.prediction_horizons:
            has_prediction = True
            pred_data = self.prediction_horizons[prediction_key]
            
            # Convert prediction times to datetime objects if needed
            pred_times = []
            for pt in pred_data['prediction_times']:
                if isinstance(pt, str):
                    try:
                        pred_times.append(datetime.strptime(pt, "%Y-%m-%d %H:%M:%S"))
                    except:
                        pred_times.append(pt)
                else:
                    pred_times.append(pt)
            
            # Get prediction data
            pred_temperatures = pred_data['predicted_temperatures']
            lower_bounds = pred_data['lower_bounds']
            upper_bounds = pred_data['upper_bounds']
            pred_heating = pred_data['heating_power']
            
            # Plot temperature predictions
            self.ax_temp.plot(pred_times, pred_temperatures, 'b--', 
                             marker='.', markersize=5, label='Predicted Temperature')
            
            # Plot temperature bounds
            self.ax_temp.fill_between(pred_times, lower_bounds, upper_bounds, 
                                     color='lightgreen', alpha=0.3, label='Comfort Zone')
            
            # Plot heating power predictions
            self.ax_power.step(pred_times, pred_heating, 'g--', where='post', 
                              marker='.', markersize=5, label='Predicted Power')
            
            # Plot energy prices if available
            if 'energy_price' in pred_data:
                self.ax_price.step(pred_times, pred_data['energy_price'], 'r--', 
                                  where='post', marker='.', markersize=5, 
                                  label='Predicted Price')
                                  
            # Plot predicted outdoor temperature if available
            if 'outdoor_temperature' in pred_data:
                self.ax_outdoor.plot(pred_times, pred_data['outdoor_temperature'], 'b--', 
                                    marker='.', markersize=5, label='Predicted Outdoor Temp')
                                    
            # Plot predicted solar radiation if available
            if 'solar_radiation' in pred_data:
                self.ax_solar.plot(pred_times, pred_data['solar_radiation'], 'y--', 
                                  marker='.', markersize=5, label='Predicted Solar')
        
        # Highlight the current timestep with vertical line only
        for ax in [self.ax_temp, self.ax_power, self.ax_price, self.ax_outdoor]:
            # Vertical line at the current time
            ax.axvline(x=current_time, color='black', linestyle='-', linewidth=1.5, alpha=0.7)
        
        # Update the timestamp text at the top of the figure
        if isinstance(current_time, datetime):
            time_display = current_time.strftime("%Y-%m-%d %H:%M")
            self.timestamp_text.set_text(f"Current Timestep: {time_display}")
        else:
            self.timestamp_text.set_text(f"Current Timestep: {current_time}")
        
        # Set fixed y-axis limits
        self.ax_temp.set_ylim(self.temp_limits)
        self.ax_power.set_ylim(self.power_limits)
        self.ax_price.set_ylim(self.price_limits)
        self.ax_outdoor.set_ylim(self.outdoor_temp_limits)
        self.ax_solar.set_ylim(self.solar_limits)
        
        # Format plots
        self.ax_temp.set_title('MPC Temperature Control', fontsize=12)
        self.ax_temp.set_ylabel('Temperature (°C)', fontsize=10)
        self.ax_temp.grid(True)
        self.ax_temp.legend(loc='best')
        
        self.ax_power.set_title('Heating Power', fontsize=12)
        self.ax_power.set_ylabel('Power (W)', fontsize=10)
        self.ax_power.grid(True)
        self.ax_power.legend(loc='best')
        
        self.ax_price.set_title('Energy Price', fontsize=12)
        self.ax_price.set_ylabel('Price', fontsize=10)
        self.ax_price.grid(True)
        self.ax_price.legend(loc='best')
        
        # Format outdoor temp and solar radiation plot with dual y-axes
        self.ax_outdoor.set_title('Outdoor Conditions', fontsize=12)
        self.ax_outdoor.set_ylabel('Outdoor Temperature (°C)', fontsize=10, color='blue')
        self.ax_outdoor.tick_params(axis='y', labelcolor='blue')
        self.ax_outdoor.grid(True)
        
        self.ax_solar.set_ylabel('Solar Radiation (W/m²)', fontsize=10, color='orange')
        self.ax_solar.tick_params(axis='y', labelcolor='orange')
        
        # Format x-axis for all plots
        self.ax_outdoor.set_xlabel('Time', fontsize=10)
        
        # Combine legends for outdoor temp and solar radiation
        lines1, labels1 = self.ax_outdoor.get_legend_handles_labels()
        lines2, labels2 = self.ax_solar.get_legend_handles_labels()
        self.ax_outdoor.legend(lines1 + lines2, labels1 + labels2, loc='best')
        
        # Hide x-axis labels for all plots except the bottom one
        plt.setp(self.ax_temp.get_xticklabels(), visible=False)
        plt.setp(self.ax_power.get_xticklabels(), visible=False)
        plt.setp(self.ax_price.get_xticklabels(), visible=False)
        
        # Format x-axis for bottom plot
        date_format = mdates.DateFormatter('%H:%M')
        self.ax_outdoor.xaxis.set_major_formatter(date_format)
        self.ax_outdoor.tick_params(axis='x', rotation=45)
        
        # Update info text
        self._update_info(index, current_time, history_index, has_prediction)
        
        # Adjust layout and draw
        self.fig.tight_layout(rect=[0, 0, 1, 0.97])  # Make space for the timestamp text at top
        self.canvas.draw()
    
    def _update_info(self, index, current_time, history_index, has_prediction):
        """Update information text."""
        # Format time
        if isinstance(current_time, datetime):
            time_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
        else:
            time_str = str(current_time)
        
        # Get current values
        current_temp = self.history['measured_temperature'][history_index]
        current_setpoint = self.history['applied_setpoint'][history_index]
        current_heating = self.history['applied_heating_power'][history_index]
        
        # Get outdoor temperature and solar radiation if available
        current_outdoor_temp = None
        if 'outdoor_temperature' in self.history:
            current_outdoor_temp = self.history['outdoor_temperature'][history_index]
            
        current_solar = None
        if 'solar_radiation' in self.history:
            current_solar = self.history['solar_radiation'][history_index]
        
        # Clear and update info text
        self.info_text.delete(1.0, tk.END)
        
        info = (
            f"Controller Step: {index+1}/{self.max_index+1} | Time: {time_str}\n"
            f"Current Temperature: {current_temp:.2f}°C | Current Setpoint: {current_setpoint:.2f}°C | "
            f"Current Heating: {current_heating:.1f} W"
        )
        
        # Add outdoor conditions if available
        if current_outdoor_temp is not None or current_solar is not None:
            info += "\n"
            if current_outdoor_temp is not None:
                info += f"Outdoor Temperature: {current_outdoor_temp:.2f}°C | "
            if current_solar is not None:
                info += f"Solar Radiation: {current_solar:.1f} W/m²"
        
        if has_prediction:
            info += f"\nPrediction data available for this timestep."
        else:
            info += f"\nNo prediction data available for this timestep."
        
        self.info_text.insert(tk.END, info)
    
    def run(self):
        """Run the visualization tool."""
        self.root.mainloop()


# Example usage if run directly
if __name__ == "__main__":
    # Create a dummy MPC object with sample data
    class DummyMPC:
        def __init__(self):
            from datetime import datetime, timedelta
            import numpy as np
            
            # Create sample timestamps (every minute for 1 day)
            start_time = datetime(2023, 1, 1, 0, 0, 0)
            timestamps = [start_time + timedelta(minutes=i) for i in range(24*60)]
            
            # Set time step
            self.time_step = 60  # 60 minutes = 1 hour
            
            # Create sample data
            self.history = {
                'timestamp': timestamps,
                'measured_temperature': [21 + np.sin(i/240) for i in range(24*60)],
                'applied_setpoint': [21 + 0.5*np.sin(i/240-0.5) for i in range(24*60)],
                'applied_heating_power': [500 + 300*np.sin(i/240-1) for i in range(24*60)],
                'energy_price': [0.15 + 0.03*np.sin(i/240) for i in range(24*60)],
                'outdoor_temperature': [10 + 5*np.sin(i/120) for i in range(24*60)],
                'solar_radiation': [max(0, 600*np.sin(np.pi*(i % 1440)/720)) for i in range(24*60)]
            }
            
            # Create prediction horizons (only at hourly steps)
            self.prediction_horizons = {}
            for i in range(0, 24*60, 60):  # Every hour
                t = timestamps[i]
                key = t.strftime("%Y-%m-%d %H:%M:%S")
                
                horizon = 6  # 6-hour prediction horizon
                pred_times = [t + timedelta(hours=j) for j in range(horizon)]
                
                # Calculate hour of day for solar predictions
                hours_of_day = [(t.hour + j) % 24 for j in range(horizon)]
                
                self.prediction_horizons[key] = {
                    'prediction_times': pred_times,
                    'predicted_temperatures': [21 + np.sin((i+j*60)/240) for j in range(horizon)],
                    'lower_bounds': [19] * horizon,
                    'upper_bounds': [23] * horizon,
                    'heating_power': [500 + 300*np.sin((i+j*60)/240-1) for j in range(horizon)],
                    'energy_price': [0.15 + 0.03*np.sin((i+j*60)/240) for j in range(horizon)],
                    'outdoor_temperature': [10 + 5*np.sin((i+j*60)/120) for j in range(horizon)],
                    'solar_radiation': [max(0, 600*np.sin(np.pi*((i + j*60) % 1440)/720)) for j in range(horizon)]
                }
    
    # Create and run visualizer
    dummy_mpc = DummyMPC()
    visualizer = MPCVisualizer(dummy_mpc)
    visualizer.run()