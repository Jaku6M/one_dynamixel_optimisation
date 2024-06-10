import xml.etree.ElementTree as ET
import os
import sys
import yaml

def change_xml_values(xml_file, damping_value, friction_value, spring_stiffness_value, spring_reference_value, stopErp_value, stopCfm_value):
    # Parse the XML file
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Modify specific numerical values in <dynamics> elements
    for dynamics in root.iter('dynamics'):
        dynamics.set('damping', str(damping_value))
        dynamics.set('friction', str(friction_value))

    # Modify specific numerical values in <gazebo> elements
    for gazebo in root.iter('gazebo'):
        for spring_stiffness in gazebo.iter('springStiffness'):
            spring_stiffness.text = str(spring_stiffness_value)
        for spring_reference in gazebo.iter('springReference'):
            spring_reference.text = str(spring_reference_value)
        for stopErp in gazebo.iter('stopErp'):
            stopErp.text = str(stopErp_value)
        for stopCfm in gazebo.iter('stopCfm'):
            stopCfm.text = str(stopCfm_value)

    # Save the modified XML file
    tree.write(xml_file)

    '''
    # OLD COLCON BUILD THAT WAS USED BUT CREATED STDERR OUTPUT NOW ITS REPLACED BY OPTIMIZATIONCOLCONBUILD.PY fucnction
    # Change directory to ~/Humanoid_workspace and build the specific package
    workspace_dir = os.path.expanduser('~/Humanoid_workspace')
    if os.path.isdir(workspace_dir):
        os.chdir(workspace_dir)
        # Run colcon build command for the specified package
        command = 'colcon build --packages-select one_dynamixel_simulation'
        os.system(command)
        print(f"Built package 'one_dynamixel_simulation' in {workspace_dir}.")
    else:
        print(f"Error: Workspace directory not found at {workspace_dir}.")
    '''

def change_yaml_values(yaml_file, p_value, i_value, d_value):
    # Load YAML file
    with open(yaml_file, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # Modify specific numerical values in YAML data
    yaml_data['joint_trajectory_controller']['ros__parameters']['gains']['joint1']['p'] = p_value
    yaml_data['joint_trajectory_controller']['ros__parameters']['gains']['joint1']['i'] = i_value
    yaml_data['joint_trajectory_controller']['ros__parameters']['gains']['joint1']['d'] = d_value

    # Save the modified YAML data back to the file
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f)

    print(f"Changed values in file {yaml_file}.")

if __name__ == "__main__":
    # Retrieve parameters from command line arguments
    damping_value = float(sys.argv[1])
    friction_value = float(sys.argv[2])
    spring_stiffness_value = float(sys.argv[3])
    spring_reference_value = float(sys.argv[4])
    stopErp_value = float(sys.argv[5])
    stopCfm_value = float(sys.argv[6])

    # Specify the path to the XML file
    xml_file_path = '/home/jaku6m/Humanoid_workspace/src/one_dynamixel_simulation/urdf/MX64AR.xacro'
    # Call the function to change XML values with received parameters
    change_xml_values(xml_file_path, damping_value, friction_value, spring_stiffness_value, spring_reference_value, stopErp_value, stopCfm_value)

    # Retrieve parameters from command line arguments for YAML file
    p_value = float(sys.argv[7])
    i_value = float(sys.argv[8])
    d_value = float(sys.argv[9])
    yaml_file_path = '/home/jaku6m/Humanoid_workspace/src/one_dynamixel_simulation/controllers/MX64AR_gazebo_controller.yaml'
    # Call the function to change YAML values with received parameters
    change_yaml_values(yaml_file_path, p_value, i_value, d_value)
