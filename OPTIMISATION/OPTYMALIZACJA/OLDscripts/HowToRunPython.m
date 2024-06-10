% Specify the full path to the directory you want to change to
workspace_directory = '/home/jaku6m/Humanoid_workspace';

% Change the current working directory to the specified workspace directory
cd(workspace_directory);

% Specify the full path to your Python script
python_script_path = '/home/jaku6m/Humanoid_workspace/src/one_dynamixel_simulation/launch/ChangeVals.py';

% Use the system function to run the Python script
status = system(['python3 ' python_script_path]);

% Check the status of the execution
if status == 0
    disp('Python script executed successfully.');
else
    disp('Error executing Python script.');
end
