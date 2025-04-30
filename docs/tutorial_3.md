# Building a Model Predictive Controller: Implementation Tutorial

## Table of Contents
1. [Introduction](#introduction)
2. [MPC Fundamentals](#mpc-fundamentals)
3. [Class Structure](#class-structure)
4. [Component Walkthrough](#component-walkthrough)
   - [Initialization](#initialization)
   - [Control Output Method](#control-output-method)
   - [Optimization Method](#optimization-method)
5. [Implementation Details](#implementation-details)
6. [Testing and Validation](#testing-and-validation)
7. [Exercises](#exercises)

## Introduction

Model Predictive Control (MPC) is an advanced control strategy that uses a mathematical model of a system to predict its future behavior and optimize control actions. In building control applications, MPC can significantly improve energy efficiency while maintaining comfort by incorporating weather forecasts, energy prices, and comfort constraints into the optimization process.

This tutorial focuses on implementing a ModelPredictiveControl class for building temperature regulation. We'll explore the core components and algorithms that make MPC work, with practical implementation details using Python and the CVXPY optimization library.

## MPC Fundamentals

Before diving into implementation, let's review the key principles of MPC:

### Core MPC Components

1. **System Model**: A mathematical representation of the system dynamics (in our case, building thermal behavior)
2. **Prediction Horizon**: The time period over which future behavior is predicted and control actions are optimized
3. **Cost Function**: The objective to minimize (typically energy consumption or cost, with penalties for constraint violations)
4. **Constraints**: Limitations on control actions and system states (comfort bounds, maximum heating power)
5. **Receding Horizon**: Only the first control action is implemented, then the optimization is repeated with updated information

### MPC Algorithm Flow

The MPC controller operates in a cyclical process:

1. Measure the current system state
2. Use the model to predict future behavior over the horizon
3. Optimize control actions to minimize the cost function while satisfying constraints
4. Apply the first control action
5. Move to the next time step and repeat

## Class Structure

Our `ModelPredictiveControl` class implements this algorithm for building temperature control. Here's the overall structure:

```python
class ModelPredictiveControl:
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
                 control_type="direct"
                 ):
        # Initialize parameters and components
        
    def get_control_output(self, current_time, simulator):
        # Main control interface that's called at each time step
        # Returns setpoint and heating power
        
    def optimization(self, solver, y_measured, innovation, 
                     A, C, K, Ba, Bs, Bh, 
                     outdoor_temp_forecast, solar_forecast, Tmin, Tmax, price_forecast):
        # Core optimization algorithm
        # Formulates and solves the convex optimization problem
```

## Component Walkthrough

Let's examine each component of the class in detail.

### Initialization

The `__init__` method sets up all the necessary components for the MPC controller:

```python
def __init__(self, 
             time_step=60,          # Control time step in minutes
             sensor_name=None,      # Name of the temperature sensor
             model=None,            # State-space model of the building
             weather_parser=None,   # Parser for weather forecast data
             grid_parser=None,      # Parser for electricity price forecast
             low_setpoint_parser=None,  # Parser for minimum temperature setpoints
             high_setpoint_parser=None, # Parser for maximum temperature setpoints
             max_power=500,         # Maximum heating power in Watts
             horizon=24,            # Prediction horizon in hours
             slack_weight=10e6,     # Penalty weight for comfort constraint violations
             solver="CLARABEL",     # CVXPY solver to use
             initial_state=None,    # Initial state vector (if None, use model.x0)
             control_type="direct"  # Control strategy type
             ):
    """
    Initialize the MPC controller with the given parameters.
    
    Args:
        time_step: Control interval in minutes
        sensor_name: Name of the temperature sensor to use
        model: State-space model of the building thermal dynamics
        weather_parser: Object that provides weather forecasts
        grid_parser: Object that provides electricity price forecasts
        low_setpoint_parser: Object that provides minimum temperature setpoints
        high_setpoint_parser: Object that provides maximum temperature setpoints
        max_power: Maximum heating power in Watts
        horizon: Prediction horizon in hours
        slack_weight: Weight for penalizing comfort constraint violations
        solver: CVXPY solver to use for optimization
        initial_state: Initial state vector (if None, use model.x0)
        control_type: Type of control strategy
    """
    
    # Set up the solver for optimization
    self.solver = solver
    
    # Store the building thermal model
    self.model = model
    self.x0 = model.x0 if initial_state is None else initial_state
    
    # Store forecast parsers for predictions
    self.weather = weather_parser            # Weather forecasts (outdoor temp, solar)
    self.grid = grid_parser                  # Electricity price forecasts
    self.low_setpoint = low_setpoint_parser  # Minimum temperature constraints
    self.high_setpoint = high_setpoint_parser # Maximum temperature constraints
    
    # Store MPC parameters
    self.control_type = control_type
    self.time_step = time_step
    self.max_power = max_power
    self.horizon = horizon
    self.slack_weight = slack_weight
    self.sensor_name = sensor_name

    # Initialize control outputs
    self.setpoint = 20       # Default temperature setpoint in °C
    self.heating_power = 0   # Initial heating power in Watts
```

**Key Initialization Parameters:**

- **time_step**: How frequently the MPC optimization runs (typically minutes)
- **model**: State-space model that captures the building's thermal dynamics
- **forecast parsers**: Components that provide predictions of weather, energy prices, and comfort constraints
- **horizon**: How far into the future the controller predicts and optimizes (typically hours)
- **slack_weight**: How strongly comfort constraint violations are penalized
- **control_type**: Whether the MPC directly controls heating power ("direct") or sets a temperature setpoint ("indirect")

### Control Output Method

The `get_control_output` method is called at each time step and determines whether to run a new optimization:

```python
def get_control_output(self, current_time, simulator):
    """
    Calculate the control output (setpoint and heating power) for the current time.
    
    This method is called at each time step. It only performs the optimization
    at intervals specified by time_step.
    
    Args:
        current_time: Current simulation time
        simulator: Simulator object that contains sensor data
        
    Returns:
        tuple: (temperature_setpoint, heating_power)
    """
    
    # Only run optimization at the specified time intervals
    # (e.g., if time_step=60, run once every hour when minutes=0)
    if current_time.minute % self.time_step == 0:
        # Get the latest indoor temperature measurement from the sensor
        y_measured = simulator.sensor_manager.sensors[simulator.high_level_control.sensor_name]["data"][-1]

        # Extract model matrices from the state-space model
        A = self.model.A  # State transition matrix
        B = self.model.B  # Input matrix
        C = self.model.C  # Output matrix
        K = self.model.K  # Kalman gain for state estimation
        
        # Extract input columns from B matrix for clarity:
        # The B matrix typically has columns for different inputs
        Ba = B[:, 0]  # Effect of ambient (outdoor) temperature on states
        Bs = B[:, 1]  # Effect of solar radiation on states
        Bh = B[:, 2]  # Effect of heating power on states
        
        # Get forecasts for the prediction horizon
        # --------------------------------------
        
        # Electricity price forecast
        price_forecast, _, _ = self.grid.get_prediction_horizon(current_time, self.horizon)
        
        # Weather forecasts (outdoor temperature and solar radiation)
        outdoor_temp_forecast, solar_forecast, _ = self.weather.get_prediction_horizon(
            current_time, self.horizon
        )
        
        # Temperature constraint forecasts (min and max allowed temperatures)
        Tmin, _ = self.low_setpoint.get_prediction_horizon(current_time, self.horizon)
        Tmax, _ = self.high_setpoint.get_prediction_horizon(current_time, self.horizon)
        
        # Kalman filter state estimation
        # -----------------------------
        # Calculate the innovation (measurement residual)
        # This is the difference between actual measurement and predicted measurement
        y_estimated = C @ self.x0.flatten(order="F")  # Predicted output based on current state
        innovation = y_measured - y_estimated         # Measurement residual
        
        # Solve the MPC optimization problem
        # ----------------------------------
        x, Qh = self.optimization(
            self.solver,
            y_measured, 
            innovation, 
            A, C, K, Ba, Bs, Bh, 
            outdoor_temp_forecast, 
            solar_forecast, 
            Tmin, Tmax, 
            price_forecast
        )
        
        # Update state estimate with first step of optimal trajectory
        # The optimization returns the entire state trajectory, but we only use
        # the first state of the next time step (x[1])
        self.x0 = x.value[1, :]
        
        # Calculate the temperature setpoint from the state
        self.setpoint = float(C @ self.x0)
        
        # Get optimal heating power for the next time step
        self.heating_power = float(Qh.value[0])
        
        # Print debugging information
        print(f"Time: {current_time}")
        print(f"Innovation (measurement residual): {innovation}")

    # Return the current setpoint and heating power
    # If this isn't an optimization step, return previous values
    return self.setpoint, self.heating_power
```

**Key Control Output Components:**

1. **Timing Logic**: MPC optimization runs at specified intervals (e.g., every hour)
2. **State Estimation**: Uses the Kalman filter approach to update state estimates based on measurements
3. **Forecast Collection**: Gathers predictions of weather, prices, and comfort constraints
4. **Optimization Call**: Passes all necessary information to the optimization method
5. **State and Output Update**: Applies the first optimal control action and updates the state estimate

### Optimization Method

The `optimization` method is the heart of MPC, where the convex optimization problem is formulated and solved:

```python
def optimization(self, solver, y_measured, innovation, 
                 A, C, K, Ba, Bs, Bh, 
                 outdoor_temp_forecast, solar_forecast, Tmin, Tmax, price_forecast):
    """
    Formulate and solve the MPC optimization problem.
    
    This method sets up the optimization problem to find the optimal
    heating power trajectory that minimizes energy cost while maintaining
    indoor temperature within comfort constraints.
    
    Args:
        solver: CVXPY solver to use
        y_measured: Current temperature measurement
        innovation: Measurement residual for state estimation
        A, C, K: State-space matrices and Kalman gain
        Ba, Bs, Bh: Input effect vectors for outdoor temp, solar, and heating
        outdoor_temp_forecast: Forecast of outdoor temperatures
        solar_forecast: Forecast of solar radiation
        Tmin: Minimum allowed temperatures
        Tmax: Maximum allowed temperatures
        price_forecast: Forecast of electricity prices
        
    Returns:
        tuple: (optimal_state_trajectory, optimal_heating_power)
    """

    n_x = A.shape[0]  # State dimension (number of state variables)
    
    # Define optimization variables
    # ----------------------------
    # State trajectory over the prediction horizon
    x = cp.Variable((self.horizon + 1, n_x))
    
    # Heating power trajectory (must be non-negative)
    Qh = cp.Variable(self.horizon, nonneg=True)
    
    # Slack variables for soft constraint implementation
    # (allows temperature constraints to be violated but with penalty)
    slack = cp.Variable(self.horizon, nonneg=True)
    
    # Set up constraints
    # -----------------
    # Start with an empty list of constraints
    constraints = []
    
    # Initial condition: x₀ is the current state estimate
    constraints.append(x[0] == self.x0.flatten(order="F"))
    
    # Initialize cost function
    cost = 0
    
    # Add dynamics and constraints for each step in the prediction horizon
    for i in range(self.horizon):
        # System dynamics constraints
        # ---------------------------
        if i == 0:
            # First step includes estimation correction using the innovation term
            # x₁ = A·x₀ + Ba·T_outdoor + Bs·solar + Bh·Q_heat + K·innovation
            constraints.append(
                x[i + 1, :] == A @ x[0, :] + 
                Ba * outdoor_temp_forecast[i] + 
                Bs * solar_forecast[i] + 
                Bh * Qh[i] + 
                K @ innovation
            )
        else:
            # Subsequent steps use the standard state-space model without correction
            # xᵢ₊₁ = A·xᵢ + Ba·T_outdoor + Bs·solar + Bh·Q_heat
            constraints.append(
                x[i + 1, :] == A @ x[i, :] + 
                Ba * outdoor_temp_forecast[i] + 
                Bs * solar_forecast[i] + 
                Bh * Qh[i]
            )
        
        # Temperature comfort constraints (with slack variables)
        # ----------------------------------------------------
        # The indoor temperature (C·x) should be between Tmin and Tmax
        # Slack variables allow these constraints to be violated at a cost
        constraints.append(Tmin[i] - slack[i] <= C @ x[i + 1, :])  # Lower bound
        constraints.append(C @ x[i + 1, :] <= Tmax[i] + slack[i])  # Upper bound
    
        # Heating power constraints
        # ------------------------
        # Heating power cannot exceed the maximum available power
        constraints.append(Qh[i] <= self.max_power)
        
        # Cost function
        # ------------
        # Minimize: Energy cost + penalty for comfort violations
        # Energy cost = heating power × electricity price
        # Comfort violation penalty = slack_weight × slack
        cost += Qh[i] * price_forecast[i] + self.slack_weight * slack[i]
    
    # Create and solve the optimization problem
    # ----------------------------------------
    objective = cp.Minimize(cost)
    problem = cp.Problem(objective, constraints)
    
    # Solve the problem using the specified solver
    problem.solve(solver=solver, verbose=False)
    
    # Return the optimal state trajectory and heating power
    return x, Qh
```

**Key Optimization Components:**

1. **Decision Variables**:
   - State trajectory (`x`): Building thermal states over the prediction horizon
   - Heating power (`Qh`): Control actions to optimize
   - Slack variables (`slack`): Allow soft constraint violations with penalties

2. **Constraints**:
   - Initial condition: Starting from the current estimated state
   - System dynamics: How the states evolve based on the model and inputs
   - Comfort bounds: Keeping temperature within acceptable ranges (with slack)
   - Power limits: Maximum heating capacity

3. **Cost Function**:
   - Energy cost: Product of heating power and electricity price
   - Comfort violation penalty: Weighted slack variables

4. **Problem Solution**:
   - Creates and solves a convex optimization problem
   - Returns the optimal state trajectory and heating power sequence

## Implementation Details

### State-Space Model

The MPC controller uses a state-space model of the building thermal dynamics:

```
x(k+1) = A·x(k) + B·u(k)
y(k) = C·x(k)
```

Where:
- `x(k)` is the state vector (temperature of building elements)
- `u(k)` is the input vector (outdoor temperature, solar radiation, heating power)
- `y(k)` is the output (indoor temperature)
- `A`, `B`, `C` are matrices that define the system dynamics

### Kalman Filter for State Estimation

Since not all states are directly measurable, we use a Kalman filter approach:

1. Predict the current output based on the state estimate: `y_estimated = C·x₀`
2. Calculate the innovation: `innovation = y_measured - y_estimated`
3. Correct the next state prediction using the innovation: `+ K·innovation`

### Slack Variables and Soft Constraints

Comfort constraints are implemented as soft constraints using slack variables:

```python
# Lower bound with slack
Tmin[i] - slack[i] <= C @ x[i + 1, :]

# Upper bound with slack
C @ x[i + 1, :] <= Tmax[i] + slack[i]
```

This allows violations when necessary but penalizes them in the cost function:
```python
cost += Qh[i] * price_forecast[i] + self.slack_weight * slack[i]
```

### Forecast Integration

The controller uses multiple forecast parsers to get predictions for:
- Weather conditions (outdoor temperature, solar radiation)
- Electricity prices
- Comfort constraints (minimum and maximum temperature bounds)

These forecasts are integrated into the optimization problem to enable predictive control.


## Exercises

1. **Model Impact Analysis**:
   - Try different state-space models
   - Observe how model accuracy affects control performance
   - Try 1st, 2nd, and 3rd order models and compare results
   - Try models training for different objectives (e.g. one-step versus k-step predictions)

2. **Cost Function Experimentation**:
   - Implement different cost function formulations:
     - Energy minimization: `cost += Qh[i] + self.slack_weight * slack[i]`
     - Cost minimization: `cost += Qh[i] * price_forecast[i] + self.slack_weight * slack[i]`
     - Comfort prioritization: `cost += Qh[i] + self.slack_weight * 10 * slack[i]`
   - Compare the resulting control strategies

3. **Horizon Length Effects**:
   - Test different prediction horizons (6, 12, 24, 48 hours)
   - Measure energy savings, comfort violations, and computation time
   - Determine the optimal horizon length for your building model
   
4. **Compare with tradional baseline controllers**:
   - Compare performance relative to a base PI controller
   - Evauate with respect to different KPI´s
