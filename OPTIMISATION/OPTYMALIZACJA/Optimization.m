clear;
clc;
RealFile = '/home/jaku6m/Desktop/Plots/RealMX28Plots/Trajectory8_SlowLongrotation.csv';
Gazebo_raw_File = "/home/jaku6m/Desktop/OPTYMALIZACJA/OptcsvGazeboFiles/feedback_data.csv";
% Define initial guess for parameters:
% [damping_value, friction_value, spring_stiffness_value, spring_reference_value, stopErp_value, stopCfm_value, p_value, i_value, d_value)]
% initial_guess = [0.2, 0.2, 1.0, 0.01];
initial_guess = [1.825, 1.288, 0.5, 0.061, 0.7, 0.45, 77.7, 1.0, 0.0];
% Set up bounds for parameters
% lower_bounds = initial_guess - 0.4 * abs(initial_guess);
% upper_bounds = initial_guess + 1.0 * abs(initial_guess);
lower_bounds = [0.1, 0.1, 0.1, 0.01, 0.1, 0.1, 20, 0.0, 0.0];
upper_bounds = [4.0, 4.0, 4.0, 4.0, 0.8, 0.8, 100.0, 1.0, 1.0];
% Run optimization
options = optimoptions('patternsearch', 'Display', 'iter');
[opt_params, fval] = patternsearch(@(params) combined_error_function(RealFile, Gazebo_raw_File, params), initial_guess, [], [], [], [], lower_bounds, upper_bounds, [], options);
% Display optimized parameters and objective value
disp('Optimized Parameters:');
disp(opt_params);
disp('Optimized Objective Value:');
disp(fval);

% The final values
[sum_position_errorsquared, sum_velocity_errorsquared] = DynamicErrorAutomatic(RealFile, Gazebo_raw_File);
disp('Sum of position error squared:');
disp(sum_position_errorsquared);
disp('Sum of velocity error squared:');
disp(sum_velocity_errorsquared);