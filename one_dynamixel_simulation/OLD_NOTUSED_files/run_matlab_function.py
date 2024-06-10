import matlab.engine

def run_matlab_function(real_file, gazebo_raw_file):
    # Start MATLAB Engine
    eng = matlab.engine.start_matlab()

    # Call the MATLAB function
    sum_position_errorsquared, sum_velocity_errorsquared = eng.DynamicErrorAutomatic(real_file, gazebo_raw_file, nargout=2)

    # Stop MATLAB Engine
    eng.quit()

    # Return the results
    return sum_position_errorsquared, sum_velocity_errorsquared

if __name__ == "__main__":
    # Specify the paths to the CSV files
    real_file = '/home/jaku6m/Desktop/Plots/RealMX28Plots/Trajectory2_9secTrajectorySTART0.csv'
    gazebo_raw_file = '/home/jaku6m/Desktop/OPTYMALIZACJA/OptcsvGazeboFiles/feedback_data.csv'

    # Call the MATLAB function
    sum_position_errorsquared, sum_velocity_errorsquared = run_matlab_function(real_file, gazebo_raw_file)

    # Print the results
    print("Sum of position error squared:", sum_position_errorsquared)
    print("Sum of velocity error squared:", sum_velocity_errorsquared)