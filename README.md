# Drone Landing on Boats with PX4 and Gazebo
Simulation of the landing of gimballed PX4 quadcopters on boats in a Gazebo wave environment using both basic heuristic and reinforcement learning techniques.

## Installation

Below are the steps to integrate the [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) world with a PX4 drone on Ubuntu 24.04:

1. Install ROS 2 Jazzy,
2. Install ros-gz, which installs Gazebo Harmonic
3. Install PX4 https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html 
4. `cd PX4-Autopilot & export GZ_VERSION=harmonic # optionally add to ~/.bashrc`
5. Install asv_wave_sim and the rs750 sailboat model:

   ```sudo apt-get update
   sudo apt-get install libcgal-dev libfftw3-dev
   sudo apt-get install -y build-essential
   sudo apt install python3-colcon-common-extensions
   sudo apt update
   sudo apt install libboost-iostreams-dev
   sudo apt install libboost-all-dev
   
   mkdir -p gz_ws/src
   cd gz_ws/src
   git clone https://github.com/srmainwaring/asv_sim.git
   git clone https://github.com/srmainwaring/asv_wave_sim.git
   git clone https://github.com/srmainwaring/rs750.git
   
   # Build
   cd ..
   colcon build --symlink-install --merge-install --cmake-args \
   -DCMAKE_BUILD_TYPE=RelWithDebInfo \
   -DBUILD_TESTING=ON \
   -DCMAKE_CXX_STANDARD=17
   
   cd src/ardupilot_gazebo
   mkdir build && cd build
   cmake .. 
   make
   
   cd ~/PX4-Autopilot/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control 
   mkdir build && cd build
   cmake .. && make
   ```

6. Add the provided waves.sdf file to the `~/PX4-Autopilot/Tools/simulation/gz/worlds` folder
   ```
   cd ~/PX4-Autopilot/Tools/simulation/gz/worlds
   touch waves.sdf
   ```
Then paste into waves.sdf the provided file (this ensures the file is created in your own directory originally)

7. Install QGroundControl https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html 
   ```
   chmod +x QGroundControl.AppImage 
   ./QGroundControl.AppImage
   ```

## Common issues/Important Notes

1. Make sure you don’t have duplicate plugins: either they are included in PX4-Autopilot/src/modules/simulation/gz_bridge/server.config file OR in your sdf file, not both; likely the server.config will already contain everything. Duplicates prevent the video stream from working.

2. Do NOT source ROS in your ~/.bashrc file, it will cause PX4 to fail to launch. Instead, what you can do is define an alias like

   ```alias ros2dev="source /opt/ros/jazzy/setup.bash && source ~/ros_venv/bin/activate”```

4. If the colcon build fails, try running
   
   ```
   sudo apt-get update
   sudo apt-get install rapidjson-dev
   sudo apt-get update
   sudo apt-get install -y build-essential
   ```

At this point, you should be able to run the wave simulation with a PX4 drone by itself. To enable PX4 and ROS integration, however, we still need to install Micro-XRCE-DDS. 

To install Micro-XRCE-DDS, run the following:

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Once installation is complete, you can run the following to start the full simulation:

```
cd ~/PX4-Autopilot/gz_ws
source ./install/setup.bash
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$(pwd)/src/rs750/rs750_gazebo/models:\
$(pwd)/src/rs750/rs750_gazebo/worlds:\
$HOME/PX4-Autopilot/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/PX4-Autopilot/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/PX4-Autopilot/gz_ws/src/asv_wave_sim/gz-waves-models/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/PX4-Autopilot/gz_ws/install/lib:\
export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/PX4-Autopilot/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
cd ..
PX4_GZ_WORLD=waves PX4_GZ_MODEL_POSE="0.3,-0.3,1.15" make px4_sitl gz_x500_gimbal
 
MicroXRCEAgent udp4 -p 8888

cd tracktor-beam
source install/setup.bash
ros2 launch precision_land precision_landing_system.launch.py

ros2 run rqt_image_view rqt_image_view

./Downloads/QGroundControl.AppImage
```


