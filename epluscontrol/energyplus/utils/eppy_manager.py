"""Utilities for working with eppy and EnergyPlus IDF files.
"""

import os

class EppyManager:
    """Manager class for eppy operations on EnergyPlus IDF files.
    
    This class handles finding the Energy+.idd file, setting up eppy,
    and providing methods to modify IDF files.
    
    Attributes:
        energyplus_dir (str): Path to the EnergyPlus installation directory.
        idf_path (str): Path to the IDF file to be modified.
        idd_path (str): Path to the Energy+.idd file.
    """
    
    def __init__(self, energyplus_dir, idf_path):
        """Initialize the EppyManager.
        
        Args:
            energyplus_dir (str): Path to the EnergyPlus installation directory.
                Used for locating the Energy+.idd file.
            idf_path (str): Path to the IDF file to be modified.
        
        Raises:
            ValueError: If the IDD file cannot be found.
        """
        self.energyplus_dir = energyplus_dir
        self.idf_path = idf_path
        self.idd_path = self._find_idd_file()
        
        # Import eppy components once during initialization
        try:
            from eppy.modeleditor import IDF, IDDAlreadySetError
            self.IDF = IDF
            self.IDDAlreadySetError = IDDAlreadySetError
        except ImportError:
            raise ImportError("The eppy package is required. Install it with 'pip install eppy'.")
        
    def _find_idd_file(self):
        """Find the Energy+.idd file in the EnergyPlus directory.
        
        Returns:
            str: Path to the IDD file.
        
        Raises:
            ValueError: If the IDD file cannot be found.
        """
        potential_idd = os.path.join(self.energyplus_dir, "Energy+.idd")
        if os.path.exists(potential_idd):
            print(f"Found IDD file at: {potential_idd}")
            return potential_idd
        else:
            raise ValueError(
                f"Energy+.idd file not found in the EnergyPlus directory: {self.energyplus_dir}. "
                "Please provide the correct EnergyPlus directory."
            )
    
    def setup_eppy(self):
        """Set up eppy with the correct IDD file.
        
        This method safely initializes eppy with the IDD file path, 
        handling the case where the IDD might already be set.
        
        Returns:
            bool: True if setup was successful, False otherwise.
        """
        try:
            # Try to set the IDD file path
            self.IDF.setiddname(self.idd_path)
            print(f"IDD file set to: {self.idd_path}")
            return True
        except self.IDDAlreadySetError as e:
            # IDD is already set, check if it's the same path
            current_idd = str(e).split(":")[-1].strip()
            if os.path.samefile(current_idd, self.idd_path):
                print(f"IDD file already set to the correct path: {current_idd}")
                return True
            else:
                print(f"Warning: IDD file was already set to {current_idd}, "
                      f"which is different from {self.idd_path}")
                print("Using the already set IDD file.")
                return True
        except Exception as e:
            print(f"Error setting up eppy: {str(e)}")
            return False

    def set_run_period(self, start_month=1, start_day=1, end_month=12, end_day=31):
        """Set the simulation run period in the EnergyPlus IDF file.
        
        Args:
            start_month (int): Start month (1-12). Defaults to 1 (January).
            start_day (int): Start day (1-31). Defaults to 1.
            end_month (int): End month (1-12). Defaults to 12 (December).
            end_day (int): End day (1-31). Defaults to 31.
        
        Returns:
            bool: True if successful, False otherwise.
        
        Raises:
            ValueError: If the run period parameters are invalid or eppy setup fails.
            IOError: If the IDF file cannot be read or written.
        """
        # Initialize eppy
        if not self.setup_eppy():
            raise ValueError("Failed to set up eppy for modifying the IDF file")
        
        # Validate input parameters
        if not (1 <= start_month <= 12 and 1 <= end_month <= 12):
            raise ValueError("Months must be between 1 and 12.")
        
        # Days per month (simplified, ignoring leap years)
        days_in_month = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
        
        if not (1 <= start_day <= days_in_month[start_month-1]):
            raise ValueError(f"Invalid start day for month {start_month}.")
        
        if not (1 <= end_day <= days_in_month[end_month-1]):
            raise ValueError(f"Invalid end day for month {end_month}.")
        
        try:
            # Load the IDF file
            idf = self.IDF(self.idf_path)
            
            # Find or create RunPeriod object
            run_periods = idf.idfobjects["RUNPERIOD"]
            if run_periods:
                run_period = run_periods[0]  # Use the first RunPeriod object
            else:
                run_period = idf.newidfobject("RUNPERIOD")
            
            # Set run period parameters
            run_period.Begin_Month = start_month
            run_period.Begin_Day_of_Month = start_day
            run_period.End_Month = end_month
            run_period.End_Day_of_Month = end_day
            
            # Save the modified IDF file
            idf.saveas(self.idf_path)
            
            print(f"Run period set from {start_month}/{start_day} to {end_month}/{end_day}")
            return True
            
        except Exception as e:
            raise IOError(f"Error modifying IDF file: {str(e)}")

    def set_location(self, latitude, longitude, time_zone=0, elevation=0):
        """Set the location information in the EnergyPlus IDF file.
        
        Args:
            latitude (float): Site latitude in degrees (-90 to 90).
            longitude (float): Site longitude in degrees (-180 to 180).
            time_zone (float, optional): Time zone relative to GMT. Defaults to 0.
            elevation (float, optional): Site elevation in meters. Defaults to 0.
        
        Returns:
            bool: True if successful, False otherwise.
        
        Raises:
            ValueError: If the location parameters are invalid or eppy setup fails.
            IOError: If the IDF file cannot be read or written.
        """
        # Initialize eppy
        if not self.setup_eppy():
            raise ValueError("Failed to set up eppy for modifying the IDF file")
        
        # Validate input parameters
        if not (-90 <= latitude <= 90):
            raise ValueError("Latitude must be between -90 and 90 degrees.")
            
        if not (-180 <= longitude <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees.")
            
        try:
            # Load the IDF file
            idf = self.IDF(self.idf_path)
            
            # Get or create location object
            site_objects = idf.idfobjects["SITE:LOCATION"]
            if site_objects:
                site = site_objects[0]
            else:
                site = idf.newidfobject("SITE:LOCATION")
                site.Name = "Site Location"
            
            # Set location parameters
            site.Latitude = latitude
            site.Longitude = longitude
            site.Time_Zone = time_zone
            site.Elevation = elevation
            
            # Save the modified IDF file
            idf.saveas(self.idf_path)
            
            print(f"Location set to: Lat {latitude}, Long {longitude}, "
                  f"Time Zone {time_zone}, Elevation {elevation}m")
            return True
            
        except Exception as e:
            raise IOError(f"Error modifying IDF file: {str(e)}")
    
    def set_simulation_timestep(self, timestep_per_hour=6):
        """Set the simulation timestep in the EnergyPlus IDF file.
        
        Args:
            timestep_per_hour (int): Number of timesteps per hour (1, 2, 3, 4, 5, 6, 10, 
                12, 15, 20, 30, or 60). Defaults to 6 (10-minute timesteps).
        
        Returns:
            bool: True if successful, False otherwise.
            
        Raises:
            ValueError: If the timestep parameter is invalid or eppy setup fails.
            IOError: If the IDF file cannot be read or written.
        """
        # Initialize eppy
        if not self.setup_eppy():
            raise ValueError("Failed to set up eppy for modifying the IDF file")
        
        # Validate input parameters
        valid_timesteps = [1, 2, 3, 4, 5, 6, 10, 12, 15, 20, 30, 60]
        if timestep_per_hour not in valid_timesteps:
            raise ValueError(f"Timestep must be one of {valid_timesteps}")
            
        try:
            # Load the IDF file
            idf = self.IDF(self.idf_path)
            
            # Get or create timestep object
            timestep_objects = idf.idfobjects["TIMESTEP"]
            if timestep_objects:
                timestep = timestep_objects[0]
            else:
                timestep = idf.newidfobject("TIMESTEP")
            
            # Set timestep parameter
            timestep.Number_of_Timesteps_per_Hour = timestep_per_hour
            
            # Save the modified IDF file
            idf.saveas(self.idf_path)
            
            minutes_per_timestep = 60 / timestep_per_hour
            print(f"Simulation timestep set to {timestep_per_hour} steps per hour "
                  f"({minutes_per_timestep} minutes per step)")
            return True
            
        except Exception as e:
            raise IOError(f"Error modifying IDF file: {str(e)}")
    
    def set_output_variables(self, variables, frequency="Timestep"):
        """Set output variables in the EnergyPlus IDF file.
        
        Args:
            variables (list): List of output variable names to include.
            frequency (str, optional): Reporting frequency. Must be "Timestep", "Hourly", 
                "Daily", "Monthly", or "RunPeriod". Defaults to "Timestep".
        
        Returns:
            bool: True if successful, False otherwise.
            
        Raises:
            ValueError: If the frequency parameter is invalid or eppy setup fails.
            IOError: If the IDF file cannot be read or written.
        """
        # Initialize eppy
        if not self.setup_eppy():
            raise ValueError("Failed to set up eppy for modifying the IDF file")
        
        # Validate input parameters
        valid_frequencies = ["Timestep", "Hourly", "Daily", "Monthly", "RunPeriod"]
        if frequency not in valid_frequencies:
            raise ValueError(f"Frequency must be one of {valid_frequencies}")
            
        try:
            # Load the IDF file
            idf = self.IDF(self.idf_path)
            
            # Remove existing output variable objects with the same names
            for var_name in variables:
                existing_vars = [obj for obj in idf.idfobjects["OUTPUT:VARIABLE"] 
                                if obj.Variable_Name.lower() == var_name.lower()]
                for var in existing_vars:
                    idf.removeidfobject(var)
            
            # Add the requested output variables
            for var_name in variables:
                output_var = idf.newidfobject("OUTPUT:VARIABLE")
                output_var.Key_Value = "*"  # All objects
                output_var.Variable_Name = var_name
                output_var.Reporting_Frequency = frequency
            
            # Save the modified IDF file
            idf.saveas(self.idf_path)
            
            print(f"Added {len(variables)} output variables with {frequency} reporting frequency")
            return True
            
        except Exception as e:
            raise IOError(f"Error modifying IDF file: {str(e)}")