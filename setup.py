from setuptools import setup, find_packages

setup(
    packages=find_packages(include=[
        'epluscontrol', 
        'epluscontrol.*', 
        'epluscontrol.energyplus.*',
	'epluscontrol.control.control_managers.*',
	'epluscontrol.control.high_level_control.*',
	'epluscontrol.control.low_level_control.*',
	'epluscontrol.control.parser.*',
	'epluscontrol.energyplus.utils.*',
	'epluscontrol.energyplus.visualization.*',]),
    include_package_data=True,
)