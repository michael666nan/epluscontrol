def plot_results(simulator, columns=None, figsize=(12, 8), subplot=True, title=None):
    """Plot selected columns from the simulation results with appropriate units.
    
    Args:
        columns (list, optional): List of column names to plot. 
        figsize (tuple, optional): Figure size as (width, height).
        subplot (bool, optional): Whether to create individual subplots for each variable.
            If False, all variables are plotted in a single graph.
        title (str, optional): Title for the figure.
        debug (bool, optional): Whether to print debug information.
    
    Returns:
        tuple: (fig, ax) - Figure and axes objects for further customization.
    """
    import matplotlib.pyplot as plt
    
    sensors = simulator.sensor_manager.sensors
    df = simulator.results_df
    
    # Create DataFrame if not already available
    if df is None:
        print("No results to be plotted")
    
    # Determine which columns to plot
    if columns is None:
        columns = [col for col in df]
    elif isinstance(columns, str):
        # If a single column name is provided, convert to list
        columns = [columns]
    
    # Get units for each column
    units = {}

    for col in columns:
        if col in sensors:
            # Check if 'unit' is directly in the sensor dictionary 
            if 'unit' in sensors[col]:
                units[col] = sensors[col]['unit']
            else:
                units[col] = ''
        elif col == 'setpoint':
            # Default for setpoint (same as indoor temperature)
            indoor_temp_unit = ''
            if 'indoor_temp' in sensors and 'unit' in sensors['indoor_temp']:
                indoor_temp_unit = sensors['indoor_temp']['unit']
            units[col] = indoor_temp_unit or '°C'
        else:
            units[col] = ''
    
    # Create figure and axes
    if subplot:
        n_plots = len(columns)
        fig, axes = plt.subplots(n_plots, 1, figsize=figsize, sharex=True)
        # Make sure axes is always a list/array
        if n_plots == 1:
            axes = [axes]
        
        # Plot each column in a separate subplot
        for i, col in enumerate(columns):
            if col in df.columns:
                df[col].plot(ax=axes[i], legend=True)
                
                # Add unit to y-label if available
                unit = units.get(col, '')
                if unit:
                    axes[i].set_ylabel(f"{col} [{unit}]")
                else:
                    axes[i].set_ylabel(col)
                
                # Force ylabel update (in case matplotlib caching is an issue)
                axes[i].yaxis.set_label_text(axes[i].get_ylabel())
                
                axes[i].grid(True, linestyle='--', alpha=0.7)
            else:
                print(f"Warning: Column '{col}' not found in results.")
        
        # Set overall title if provided
        if title:
            fig.suptitle(title)
        
        # Only set x-label on bottom subplot
        axes[-1].set_xlabel('Time')
        
        plt.tight_layout()
        if title:
            plt.subplots_adjust(top=0.9)  # Make room for the title
    
    else:
        # Plot all selected columns on a single graph
        fig, ax = plt.subplots(figsize=figsize)
        
        for col in columns:
            if col in df.columns:
                # Create label with unit if available
                unit = units.get(col, '')
                label = f"{col} [{unit}]" if unit else col
                
                df[col].plot(ax=ax, label=label)
            else:
                print(f"Warning: Column '{col}' not found in results.")
        
        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        
        if title:
            ax.set_title(title)
        
        plt.tight_layout()
    
    # Force draw to make sure labels are updated
    plt.draw()
    
    return fig, axes if subplot else ax