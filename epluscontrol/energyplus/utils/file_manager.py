"""
Class to manage files during cosimulation.
"""


class FileManager:
    """
    Manager class for file handling.
    
    """
    
    def __init__(self, idf, epw, energyplus_dir, output_dir):
        """
        Initialize the FileManager.
        
        Args:
            energyplus_dir (str): Path to the EnergyPlus installation directory.
                Used for locating the Energy+.idd file.
            idf_path (str): Path to the IDF file to be modified.
        
        Raises:
            ImportError: If eppy package is not installed.
            ValueError: If the IDD file cannot be found.
        """
        self.IDF_FILE = idf
        self.WEATHER_FILE = epw        
        self.OUTPUT_DIR = output_dir or "EnergyPlus Output"
        self.ENERGYPLUS_DIR = energyplus_dir
        
    
    def get_idf_path(self):
        return self.IDF_FILE
    
    def get_epw_path(self):
        return self.WEATHER_FILE
    
    def get_output_dir(self):
        return self.OUTPUT_DIR
    
    def get_energyplus_dir(self):
        return self.ENERGYPLUS_DIR