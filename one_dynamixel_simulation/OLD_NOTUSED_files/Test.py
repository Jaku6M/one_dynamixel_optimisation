import matlab.engine
import numpy as np
import pandas as pd
from scipy.optimize import minimize

# Start MATLAB engine
eng = matlab.engine.start_matlab()

# Define combined error function
def combined_error_function(params, RealFile, Gazebo_raw_File, eng):
    sum_position_errorsquared, sum_velocity_errorsquared = eng.DynamicErrorAutomatic(RealFile, Gazebo_raw_File, nargout=2)
    combined_error = 10*sum_position_errorsquared + sum_velocity_errorsquared
    return combined_error

# Define file paths
RealFile = '/home/jaku6m/Desktop/Plots/RealMX28Plots/Trajectory2_9secTrajectorySTART0.csv'
Gazebo_raw_File = "/home/jaku6m/Desktop/OPTYMALIZACJA/OptcsvGazeboFiles/feedback_data.csv"

# Define initial guess for parameters
initial_guess = [0.2, 0.2, 1.0, 0.01]

# Define bounds for parameters (+-40% of initial guess)
bounds = [(x - 0.4 * abs(x), x + 0.4 * abs(x)) for x in initial_guess]

# Run optimization
result = minimize(combined_error_function, initial_guess, args=(RealFile, Gazebo_raw_File, eng), bounds=bounds, method='Powell')

# Display optimization results
opt_params = result.x
fval = result.fun
print('Optimized Parameters:')
print(opt_params)
print('Optimized Objective Value:')
print(fval)


# Stop MATLAB engine
eng.quit()
