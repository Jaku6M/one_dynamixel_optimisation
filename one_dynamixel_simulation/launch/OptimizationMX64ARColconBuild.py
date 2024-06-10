import os
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import xml.etree.ElementTree as ET

class MyHandler(FileSystemEventHandler):
    def __init__(self):
        self.last_modified = 0

    def on_modified(self, event):
        current_time = time.time()
        if event.src_path.endswith("MX64AR.xacro") and current_time - self.last_modified > 1:
            self.last_modified = current_time
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

def monitor_file_changes():
    # Path to the file to monitor
    file_path = '/home/jaku6m/Humanoid_workspace/src/one_dynamixel_simulation/urdf/MX64AR.xacro'
    # Create a watchdog observer
    observer = Observer()
    observer.schedule(MyHandler(), path=os.path.dirname(file_path))
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

if __name__ == "__main__":
    monitor_file_changes()
