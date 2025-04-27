import cvxpy as cp
import numpy as np
from datetime import datetime, timedelta


class ModelPredictiveControl:
    """Model Predictive Controller (MPC) for building temperature control.
    
    This controller implements a model predictive control strategy that optimizes
    heating power based on:
    - Weather forecasts (outdoor temperature, solar radiation)
    - Energy price forecasts
    - Comfort constraints (min/max temperature bounds)
    - Building thermal dynamics (state-space model)
    
    The MPC formulates and solves an optimization problem at each control interval
    to determine the optimal heating power trajectory over a prediction horizon,
    and applies the first control action of this trajectory.
    
    Args:
        time_step (int): 
            Control update interval in minutes (currently only supports 60).
        sensor_name (str): 
            Name of the temperature sensor.
        model (object): 
            State-space model of building thermal dynamics.
        weather_parser (object): 
            Parser for weather forecast data.
        grid_parser (object): 
            Parser for energy grid data.
        low_setpoint_parser (object): 
            Parser for minimum temperature bounds.
        high_setpoint_parser (object): 
            Parser for maximum temperature bounds.
        max_power (float): 
            Maximum heating power in Watts.
        horizon (int): 
            Prediction horizon length.
        slack_weight (float): 
            Weight to penalize setpoint constraint violations.
        solver (str): 
            Optimization solver ("CLARABEL", "GUROBI", etc.)
        initial_state (np.array): 
            Initial state vector (defaults to model.x0).
        print_output (bool):
            Whether to print simulation results
        correct_setpoint (bool):
            Whether to correct setpoint or not (defaults to False)
        control_type (str):
            Whether to use direct or indirect control (with low-level controller).
            Defaults to "direct"
        
    """
    
    def __init__(self, 
                 time_step=60,
                 sensor_name=None,
                 model=None,
                 weather_parser=None, 
                 grid_parser=None, 
                 low_setpoint_parser=None,
                 high_setpoint_parser=None, 
                 max_power=500, 
                 horizon=24,
                 slack_weight=10e6,
                 solver="CLARABEL",
                 initial_state=None,
                 print_output=True,
                 correct_setpoint=False,
                 control_type='direct'):
        
        # Set up the solver
        self._configure_solver(solver)
        
        # Store MPC parameters
        self.time_step = time_step
        self.max_power = max_power
        self.horizon = horizon
        self.slack_weight = slack_weight
        self.sensor_name = sensor_name
        self.correct_setpoint = correct_setpoint
        self.control_type = control_type
        
        # Store data parsers
        self.weather = weather_parser
        self.grid = grid_parser
        self.low_setpoint = low_setpoint_parser
        self.high_setpoint = high_setpoint_parser
        
        # Store model and initialize state
        self.model = model
        self.setpoint = 20  # Default setpoint
        self.heating_power = 0
        self.x0 = model.x0 if initial_state is None else initial_state
        
        # Configure data storage
        self._initialize_data_storage()
        
        # Print model information
        self._print_model_info()
        
        # Print simulation results
        self.print_output = print_output
        
        # timestep_counter
        self.counter = 0
    
    def _configure_solver(self, solver):
        """Set up the optimization solver with error checking."""
        available_solvers = cp.installed_solvers()
        
        if solver not in available_solvers:
            print(f"Warning: Requested solver '{solver}' is not available.")
            print(f"Available solvers: {', '.join(available_solvers)}")
            print("Falling back to CLARABEL solver.")
            self.solver = "CLARABEL"
        else:
            self.solver = solver
            print(f"Using {solver} solver for optimization.")
    
    def _initialize_data_storage(self):
        """Initialize data structures for storing history and predictions."""
        # For storing history of actual values
        self.history = {
            'timestamp': [],
            'measured_temperature': [],
            'innovation': [],
            'applied_setpoint': [],
            'applied_heating_power': [],
            'outdoor_temperature': [],
            'solar_radiation': [],
            'energy_price': []
        }
        
        # For storing prediction horizons
        self.prediction_horizons = {}
    
    def _print_model_info(self):
        """Print the state-space model parameters."""
        print("State-Space Model Parameters:")
        print(f"A matrix:\n{self.model.A}")
        print(f"B matrix:\n{self.model.B}")
        print(f"C matrix:\n{self.model.C}")
        print(f"D matrix:\n{self.model.D}")
        print(f"K matrix:\n{self.model.K}")
        print(f"Initial state x0:\n{self.x0}")
    
    def _store_current_values(self, current_time, y_measured, heat_measured, innovation):
        """Store current measurements in history."""
        if self.counter == 0:
            current_outdoor_temp, current_solar_rad = 0,0
            current_energy_price, current_co2 = 0,0
        else:  
            previous_time = current_time - timedelta(seconds=self.time_step * 60)         
            current_outdoor_temp, current_solar_rad = self.weather.get_values(previous_time)
            current_energy_price, current_co2 = self.grid.get_values(previous_time)

        # Store in history
        self.history['timestamp'].append(current_time)
        self.history['measured_temperature'].append(y_measured)
        self.history['innovation'].append(innovation)
        self.history['applied_setpoint'].append(self.setpoint)
        self.history['outdoor_temperature'].append(current_outdoor_temp)
        self.history['solar_radiation'].append(current_solar_rad)
        self.history['energy_price'].append(current_energy_price)
        self.history['applied_heating_power'].append(heat_measured)  # Will be updated after optimization
        
    
    def _get_model_matrices(self):
        """Extract and prepare model matrices for optimization."""
        A = self.model.A
        B = self.model.B
        C = self.model.C 
        K = self.model.K
        
        # Extract input columns for readability
        Ba = B[:, 0]  # Ambient temperature impact
        Bs = B[:, 1]  # Solar radiation impact
        Bh = B[:, 2]  # Heating power impact
        
        return A, B, C, K, Ba, Bs, Bh
    
    def _get_current_measurements(self, simulator):
        # Get mean heat from prev timestep
        heat_data = simulator.sensor_manager.sensors["Heat Power"]["data"]
        samples = min(simulator.high_level_control.time_step, len(heat_data))
        recent_values = heat_data[-samples:]
        heat_power = sum(recent_values) / len(recent_values)
        
        # Get current measurement of controlled variable
        y_measured = simulator.sensor_manager.sensors[simulator.high_level_control.sensor_name]["data"][-1]
        
        return y_measured, heat_power
    
    def _get_prediction_data(self, current_time):
        """Get forecast data for the prediction horizon."""
        # Get energy cost forecast
        energy_cost, _, prediction_times = self.grid.get_prediction_horizon(
            current_time, self.horizon)
        
        # Get weather forecast
        Ta, Qs, _ = self.weather.get_prediction_horizon(
            current_time, self.horizon)
        
        # Get temperature constraints
        Tmin, _ = self.low_setpoint.get_prediction_horizon(
            current_time, self.horizon)
        Tmax, _ = self.high_setpoint.get_prediction_horizon(
            current_time, self.horizon)
        
        return energy_cost, prediction_times, Ta, Qs, Tmin, Tmax
    
    def _formulate_optimization_problem(self, y_measured, innovation, A, C, K, Ba, Bs, Bh, 
                                      Ta, Qs, Tmin, Tmax, energy_cost):
        """Formulate the MPC optimization problem."""
        n_x = A.shape[0]  # State dimension
        
        # Define optimization variables
        x = cp.Variable((self.horizon + 1, n_x))       # State trajectory
        Qh = cp.Variable(self.horizon, nonneg=True)    # Heating power
        slack = cp.Variable(self.horizon, nonneg=True) # Constraint relaxation
        
        # Initial condition and state estimation
        constraints = [x[0] == self.x0.flatten(order="F")]
        
        # Build cost function and constraints
        cost = 0
        for i in range(self.horizon):
            # System dynamics constraints
            if i == 0:
                # First step includes estimation correction
                constraints.append(
                    x[i + 1, :] == A @ x[0, :] + 
                    Ba * Ta[i] + Bs * Qs[i] + Bh * Qh[i] + 
                    K @ innovation
                )
            else:
                constraints.append(
                    x[i + 1, :] == A @ x[i, :] + 
                    Ba * Ta[i] + Bs * Qs[i] + Bh * Qh[i]
                )
            
            # Temperature comfort constraints with slack variables
            constraints.append(Tmin[i] - slack[i] <= C @ x[i + 1, :])
            constraints.append(C @ x[i + 1, :] <= Tmax[i] + slack[i])
        
            # Heating power constraints
            constraints.append(Qh[i] <= self.max_power)
            
            # Cost function: energy cost + penalty for comfort violations
            cost += Qh[i] * energy_cost[i] + self.slack_weight * slack[i]
        
        # Create the problem
        objective = cp.Minimize(cost)
        problem = cp.Problem(objective, constraints)
        
        return problem, x, Qh, slack
    
    def _store_prediction_data(self, current_time, x, Qh, slack, C, 
                             prediction_times, Tmin, Tmax, Ta, Qs, energy_cost):
        """Store the optimization results for later analysis."""
        # Calculate predicted temperatures
        predicted_temperatures = [float(C @ x.value[i, :]) for i in range(0, self.horizon)]
        
        # # Store the optimal heating power in history
        # self.history['applied_heating_power'][-1] = float(Qh.value[0])
        
        # Create timestamp string for indexing
        timestamp_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Store the full prediction horizon
        self.prediction_horizons[timestamp_str] = {
            'prediction_times': prediction_times,
            'predicted_temperatures': predicted_temperatures,
            'heating_power': Qh.value,
            'lower_bounds': Tmin,
            'upper_bounds': Tmax,
            'outdoor_temperature': Ta,
            'solar_radiation': Qs,
            'energy_price': energy_cost,
            'slack_values': slack.value
        }
    
    def get_control_output(self, current_time, simulator, *args, **kwargs):
        """Calculate optimal temperature setpoint and heating power.
        
        Args:
            current_time (datetime): Current timestamp.
            y_measured (float): Current measured zone temperature.
            
        Returns:
            tuple: (temperature_setpoint, heating_power)
        """
        
        # Only run optimization at the specified time intervals
        if current_time.minute % self.time_step == 0:
            # Get current measurements
            y_measured, heat_measured = self._get_current_measurements(simulator)
            
            # Get model matrices
            A, B, C, K, Ba, Bs, Bh = self._get_model_matrices()
            
            # Get forecast data
            energy_cost, prediction_times, Ta, Qs, Tmin, Tmax = self._get_prediction_data(current_time)
            
            # Calculate innovation
            y_estimated = C @ self.x0.flatten(order="F")
            innovation = y_measured - y_estimated
            
            # Formulate and solve the optimization problem
            problem, x, Qh, slack = self._formulate_optimization_problem(
                y_measured, innovation, A, C, K, Ba, Bs, Bh, Ta, Qs, Tmin, Tmax, energy_cost)
            
            # Solve the problem
            try:
                problem.solve(verbose=False, solver=self.solver)
            except Exception as e:
                print(f"Error with {self.solver} solver: {e}")
                if self.solver != "CLARABEL":
                    print("Falling back to CLARABEL")
                    self.solver = "CLARABEL"
                    problem.solve(verbose=False, solver=self.solver)
            
            # Check if the solution is valid
            if problem.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Warning: MPC optimization problem status: {problem.status}")
                return self.setpoint, self.heating_power  # Return previous values
            
            # Update state estimate with first step of optimal trajectory
            self.x0 = x.value[1, :]
            
            # Calculate setpoint and heating power
            self.setpoint = float(C @ self.x0)
            self.heating_power = float(Qh.value[0])
            
            # Bound setpoint within comfort limits
            if self.correct_setpoint and self.control_type != "direct":
                self.setpoint = self._correct_setpoint(self.setpoint, Tmin[0], Tmax[0], self.heating_power)
            
            # Store current measurements in history
            self._store_current_values(current_time, y_measured, heat_measured, innovation)
            
            # Store prediction data if needed
            self._store_prediction_data(
                current_time, x, Qh, slack, C, prediction_times,
                Tmin, Tmax, Ta, Qs, energy_cost)
            
            if self.print_output: 
                print(f"Output estimation error = {innovation}")
                print(f"Optimal MPC setpoint: {self.setpoint:.2f}°C")
                print(f"Optimal MPC heat output: {self.heating_power:.2f}W")
                
            # Update counter
            self.counter += 1
        
        # Return current values
        return self.setpoint, self.heating_power
    
    def _correct_setpoint(self, setpoint, Tmin, Tmax, Qh_opt):
        if Qh_opt <= 0.0001:
            corrected_setpoint = Tmin - 0.5
        elif Qh_opt >= self.max_power-0.0001:
            corrected_setpoint = Tmax + 0.5
        else:
            corrected_setpoint = np.clip(setpoint, Tmin, Tmax)
        return corrected_setpoint