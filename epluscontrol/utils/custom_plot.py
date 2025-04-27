def plot_smart_building_data(df, figsize=(12, 10)):
    """
    Create a specialized plot for smart building data with a specific layout.
    
    Args:
        df: DataFrame containing smart building data
        figsize: Figure size (width, height)
        
    Returns:
        fig, axes: The figure and axes objects
    """
    import matplotlib.pyplot as plt
    
    # Create figure with 4 subplots
    fig, axes = plt.subplots(4, 1, figsize=figsize, sharex=True)
    
    # Plot 1: Indoor Temperature and Setpoints
    if 'Indoor Temp' in df.columns:
        df['Indoor Temp'].plot(ax=axes[0], color='blue', label='Indoor Temp')
        
        # Check and plot setpoint columns
        if 'setpoint' in df.columns:
            df['setpoint'].plot(ax=axes[0], color='red', linestyle='-.', label='Setpoint')
        
        # Also plot lower and upper setpoints if they exist (regardless of setpoint)
        if 'lower_setpoint' in df.columns:
            df['lower_setpoint'].plot(ax=axes[0], color='orange', linestyle='--', label='Lower Setpoint')
        
        if 'upper_setpoint' in df.columns:
            df['upper_setpoint'].plot(ax=axes[0], color='purple', linestyle='--', label='Upper Setpoint')
            
        # Fill between lower and upper setpoints if both exist
        if 'lower_setpoint' in df.columns and 'upper_setpoint' in df.columns:
            axes[0].fill_between(df.index, df['lower_setpoint'], df['upper_setpoint'], 
                              color='orange', alpha=0.2)
        
        axes[0].set_ylabel('Temperature [°C]')
        axes[0].set_title('Indoor Temperature and Setpoints')
        axes[0].grid(True, linestyle='--', alpha=0.7)
        axes[0].legend(loc='best')
    
    # Rest of the function remains the same
    # Plot 2: Outdoor Temperature and Solar Gain
    if 'Outdoor Temp' in df.columns:
        df['Outdoor Temp'].plot(ax=axes[1], color='blue', label='Outdoor Temp')
        axes[1].set_ylabel('Temperature [°C]')
        axes[1].set_title('Outdoor Temperature and Solar Gain')
        axes[1].grid(True, linestyle='--', alpha=0.7)
        axes[1].legend(loc='upper left')
        
        # Solar gain on right y-axis if it exists
        if 'Solar Gain' in df.columns:
            ax2 = axes[1].twinx()
            df['Solar Gain'].plot(ax=ax2, color='orange', label='Solar Gain')
            ax2.set_ylabel('Solar Gain [W]')
            
            # Get handles and labels from both axes for a unified legend
            lines1, labels1 = axes[1].get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            
            # Remove the original legend from the left axis
            axes[1].get_legend().remove()
    
    # Plot 3: Heat Power
    if 'Heat Power' in df.columns:
        df['Heat Power'].plot(ax=axes[2], color='red', label='Heat Power')
        axes[2].set_ylabel('Power [W]')
        axes[2].set_title('Heat Power')
        axes[2].grid(True, linestyle='--', alpha=0.7)
        axes[2].legend(loc='best')
    
    # Plot 4: Price and CO2
    if 'price' in df.columns:
        df['price'].plot(ax=axes[3], color='green', label='Price')
        axes[3].set_ylabel('Price [DKK/kWh]')
        axes[3].set_title('Energy Price and CO2 Emissions')
        axes[3].grid(True, linestyle='--', alpha=0.7)
        axes[3].legend(loc='upper left')
        
        # CO2 on right y-axis if it exists
        if 'co2' in df.columns:
            ax4 = axes[3].twinx()
            df['co2'].plot(ax=ax4, color='black', linestyle='--', label='CO2')
            ax4.set_ylabel('CO2 [g/kWh]')
            
            # Combined legend
            lines1, labels1 = axes[3].get_legend_handles_labels()
            lines2, labels2 = ax4.get_legend_handles_labels()
            ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            
            # Remove the original legend from the left axis
            axes[3].get_legend().remove()
    
    # Set x label on the bottom subplot only
    axes[3].set_xlabel('Time')
    
    # Adjust layout
    plt.tight_layout()
    
    return fig, axes