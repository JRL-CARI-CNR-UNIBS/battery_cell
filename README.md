# Battery Cell

## Setup

1. `cd ${HOME}` # Navigate to the home directory of the current user.
2. `mkdir -p ${HOME}/projects/battery_cell_ws/src` # Create the directory structure for the project if it doesn't exist.
3. `cd ${HOME}/projects/battery_cell_ws/src` # Change to the source directory of the project.
4. `sudo apt update -y && sudo apt upgrade -y && sudo apt install wget -y` # Update the system and install wget if it's not already installed.
5. `wget --backups=1 https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/battery_cell/demo/deps.repos` # Download the repository file.
6. `vcs import < deps.repos` # Import dependencies from the repository file. Install vcstool with `sudo apt-get install python3-vcstool` if needed.
7. **Install further dependencies** # Install required libraries and tools for the project.
    1. `sudo apt install libmosquitto-dev liblog4cxx-dev libboost-all-dev libczmq-dev liburdfdom-dev liburdfdom-headers-dev -y` # Install development libraries for MQTT, logging, Boost, ZMQ, and URDF.
    2. `sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit-visual-tools ros-humble-navigation2 -y` # Install ROS (Robot Operating System) packages.

    3. **Install ethercat (EtherLab library):** # Set up the EtherLab library for EtherCAT communication.
        1. `cd ${HOME}` # Return to the user's home directory.
        2. `mkdir -p ${HOME}/projects/software_modules` # Create a directory for software modules if it doesn't exist.
        3. `cd ${HOME}/projects/software_modules` # Change to the software modules directory.
        4. Follow the instructions at [EtherLab Installation Guide](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html) (Chapter: Installing EtherLab)
            - **Note:** When configuring the network adapter for EtherCAT, use `ifconfig` to find the correct network adapter.

    4. **Install ethercat_driver_ros2:** # Install the ROS 2 driver for EtherCAT devices.
        1. `cd ${HOME}/projects/battery_cell_ws/` # Navigate back to the workspace directory.
        2. Follow the instructions at [ethercat_driver_ros2 Installation Guide](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html) (Chapter: Building ethercat_driver_ros2)
        3. Ensure all packages are correctly compiled by running `colcon build ...` without errors (warnings are acceptable).

8. **Install Aria** # Install Aria software in the project.
    1. `cd ${HOME}/projects/battery_cell_ws/src/Aria/` # Change to the Aria directory within the project.
    2. `bash config.sh` # Run the configuration script.
    3. Insert user password if prompted.
9. `cd ${HOME}/projects/battery_cell_ws && colcon build --symlink-install --continue-on-error --cmake-args -DUSE_ROS1=False` # Build the entire project with ROS 2 settings.
   1.  if some packages fail to build, try `source ${HOME}/projects/battery_cell_ws/install/setup.bash` and then repeat step 9
10. `echo "source ${HOME}/projects/battery_cell_ws/install/setup.bash" >> ~/.bashrc`
11. `echo 'export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"; export IK_SOLVER_LOGGER_CONFIG_PATH="${HOME}/projects/battery_cell_ws/install/ik_solver/share/ik_solver/config/default_logger.yaml"' >> ~/.bashrc`
12. `source ~/.bashrc`
