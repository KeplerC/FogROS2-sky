# FogROS2-Sky

FogROS2-Sky is a cost-efficient open source robotics platform that offloads unmodified ROS2 [[1]](#1) applications to multiple cloud providers and enables fine-grained cost analysis for ROS2 applications’ communication with multiple cloud providers. As each provider offers different options for CPU, GPU, memory, and latency, it can be very difficult for users to decide which to choose. FogROS2-Sky includes an optimization algorithm, which either finds the best available hardware specification that fulfills the user’s latency and cost constraints or reports that such a specification does not exist. We use FogROS2-Sky to perform time-cost analysis on three robotics applications: visual SLAM, grasp planning. and motion planning. We are able to sample different hardware setups at nearly half the cost while still create cost and latency functions suitable for the optimizer. We also evaluate the optimizer’s efficacy for these applications with the Pareto frontier and show that the optimizer selects efficient hardware configurations to balance cost and latency. Videos and code are available [here](https://sites.google.com/view/fogros2-sky).

- [FogROS2](#fogros2)
  - [Sky Setup Instructions](#sky-setup-instructions)
  - [Usage](#usage)
  - [Installation](#installation)
    - [Quickstart](#quickstart)
    - [Docker (Recommended)](#docker-recommended)
    - [Natively](#natively)
  - [Launching ROS 2 Computational Graphs in the Cloud](#launching-ros-2-computational-graphs-in-the-cloud)
    - [Docker (Recommended)](#docker-recommended-1)
    - [Natively](#natively-1)
  - [Running Your Own Robotics Applications](#running-your-own-robotics-applications)
  - [Setting Up Automatic Image Transport](#setting-up-automatic-image-transport)
  - [Command Line Interface](#command-line-interface)
  - [Common Issues](#common-issues)
  - [Running Examples](#running-examples)
  - [References](#references)


## Sky Setup Instructions

1. Create `/sky_ws/src` and clone the repositories.

```
mkdir -p ~/sky_ws/src
cd ~/sky_ws/src
git clone https://github.com/KeplerC/FogROS2-sky.git -b spotcost
git clone https://github.com/KeplerC/rt-fogROS2 -b docker
```

2. Build the packages in the `sky_ws` directory.
```
cd ~/sky_ws
colcon build
```

3. Install `skypilot` and dependencies.
```
pip install skypilot
pip3 install boto3 
sudo apt update
```

4. Follow the installation instructions for SGC [here](https://github.com/KeplerC/fogros2-sgc).

5. Configure AWS.
```
aws configure
sky check # to make sure AWS is enabled
```

## Usage
1. To ensure Sky was set up successfully, try running the provided examples.
```
source install/setup.bash
ros2 launch fogros2_examples service.sky.launch.py 
```


### Adapt your application
1. copy and modify configuration file at `./rt-fogros2/sgc_launch/configs/service-client.yaml` (four places to edit)
2. modify the launch script at `FogROS2-sky/fogros2_examples/launch/service.sky.launch.py`  (4 steps to edit)
3. `colcon build` and launch it 


### Other useful commands
2. Check the status. 
```
sky status 
```

3. Turn off.
```
sky down sky-fogros
```




## Installation
### Quickstart
- If you are new to ROS and Ubuntu, and want to install FogROS 2 (and ROS 2) and its requisites from scratch, follow instructions [here](https://github.com/BerkeleyAutomation/FogROS2/blob/humble/QUICKSTART.md).

### Docker (Recommended)
- Alternatively, you can simplify reproduction using an OS virtualization environment with Docker. You can also watch our video tutorial [here](https://www.youtube.com/embed/oEnmZXojkcI?start=1&end=800). 
```bash
git clone -b humble https://github.com/BerkeleyAutomation/FogROS2
cd FogROS2

# Install AWS CLI
sudo apt install awscli

# Configure AWS Basic Settings. To run the next command, you need to have your security credentials, an output format and AWS Region. See https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html
aws configure

#Build Docker Image
docker build -t fogros2 .
```
- By default, this command will build a docker image for ROS Rolling and Ubuntu 22.04 (jammy). These defaults can be changed using the `--build-arg` flag (e.g., `docker build -t fogros2:focal-humble . --build-arg UBUNTU_DISTRO=focal --build-arg ROS_DISTRO=humble` will build a ROS Humble image with Ubuntu 20.04 (focal)).
*Note: the Dockerfile is cooked for x86_64. If you're using a workstation with an Arm-based architecture (e.g. an M1), build the container with the `docker build --platform linux/amd64 -t fogros2 .`*.

### Natively
- `FogROS2` is actually a ROS meta-package, so you can just fetch it in your workspace, build it, source the workspace as an overlay and start using its capabilities. You can also watch our video tutorial [here](https://www.youtube.com/embed/JlV4DhArb8Q?start=1&end=402). 

- Install ROS 2 dependencies.
```
# If using Ubuntu 22.04
sudo apt install ros-rolling-rmw-cyclonedds-cpp
```

- Install FogROS 2 dependencies.
```
sudo apt install python3-pip wireguard unzip
sudo pip3 install wgconfig boto3 paramiko scp
sudo apt install awscli
```

- Configure AWS. To run the next command, you need to have your security credentials, an output format, and an AWS Region. See https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html
```
aws configure
```

- Re-build the workspace.

```bash
source /opt/ros/<your-ros2-distro>/setup.bash
mkdir -p ~/fog_ws/src
cd ~/fog_ws/src
git clone -b humble https://github.com/BerkeleyAutomation/FogROS2
cd ../
colcon build
source install/setup.bash
```


## Launching ROS 2 Computational Graphs in the Cloud

### Docker (Recommended)
- You can see this in our video tutorial [here](https://www.youtube.com/embed/oEnmZXojkcI?start=801).

```bash
# launch fogros2 container
docker run -it --rm --net=host -v $HOME/.aws:/root/.aws --cap-add=NET_ADMIN fogros2

# launch talker node on the cloud
ros2 launch fogros2_examples talker.aws.launch.py
```

- *Note: the Dockerfile is cooked for x86_64. If you're using a workstation with an Arm-based architecture (e.g. an M1), run the container with the `docker run -it --platform linux/amd64 --rm --net=host --cap-add=NET_ADMIN fogros2`*).

### Natively
- These commands must be run from the root of your ROS workspace. You can see this in our video tutorial [here.](https://www.youtube.com/embed/JlV4DhArb8Q?start=403)
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/fogros2/share/fogros2/configs/cyclonedds.ubuntu.$(lsb_release -rs | sed 's/\.//').xml

ros2 launch fogros2_examples talker.aws.launch.py
```

## Running Your Own Robotics Applications
If using, for example, Docker, take the following steps: 

1. Mount your robotics application to docker's folder. You can use the following example or  `git clone` your development repo to the docker container instead.
```
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
       ....
    -v FOLDER_IN_YOUR_LOCAL_DIR:/home/root/fog_ws/src/YOUR_PKG_NAME \
       ...
    keplerc/ros2:latest /bin/bash
```

2.  Write the FogROS2 launch file. Examples of launch files can be found in the talker*.launch.py [here](https://github.com/BerkeleyAutomation/FogROS2/tree/humble/fogros2_examples/launch).


## Setting Up Automatic Image Transport
1. Identify all topics that need to use a compressed transport.

2. In a `fogros2.CloudNode`, add the parameter `stream_topics=[]`, where `stream_topics` is a list of tuples where each tuple is just a pair of `(TOPIC_NAME, TRANSPORT_TYPE)` values.
    - `TOPIC_NAME` is the string that represents the name of the topic that publishes `sensor_msgs/Image`
    - Valid `TRANSPORT_TYPE` values are `compressed`, `theora`, and `raw` if only `image-transport` and `image-transport-plugins` are installed on the system. 
    - `h264` is another valid `TRANSPORT_TYPE` if step 3 is followed.

3. (Optional): If using H.264, please also clone the H.264 decoder found [here](https://github.com/clydemcqueen/h264_image_transport) into the workspace's src directory. The current repo only contains the encoder and the full image transport pipeline will not work without the decoder either. Example of `stream_topics` argument:
    - `stream_topics=[('/camera/image_raw', 'h264'), ('/camera2/image_raw', 'compressed')]`
    - Adding the above argument to a `fogros2.CloudNode` makes the topic `/camera/image_raw` publish using the `h264 image_transport`, and makes the topic `/camera2/image_raw` publish using the `compressed image_transport`.
    - Please note that all cloud nodes that are expecting raw images will be remapped to `TOPIC_NAME/cloud` to remove any topic naming conflicts. (TODO: Automate remapping)

## Command Line Interface
We currently support the following CLI commands for easier debugging and development.

```bash
# List existing FogROS instances
ros2 fog list

# Connect via SSH to the corresponding instance (e.g., named "ascent-corona")
# the instance name can be found by the list command above
ros2 fog connect ascent-corona

# delete the existing FogROS instance (e.g. named "ascent-corona")
ros2 fog delete ascent-corona
# or all of the existing instances
ros2 fog delete all
```

## Common Issues
1. Warning: _2 packages has stderr outputs: fogros2 fogros2_examples_ after running colcon build. This warning occurs in Ubuntu 22.04 (jammy) builds, but does not affect functionality. See https://github.com/BerkeleyAutomation/FogROS2/issues/45. Your installation should still work.  
2. _[WARN] [1652044293.921367226] [fogros2.scp]: [Errno None] Unable to connect to port 22 on xx.xx.xx.xxx, retrying..._ . This warning occurs when AWS has not yet started the instance. This message should eventually be replaced by _SCP Connected!_ once the instance is started.
3. _WARNING: Running pip as the 'root' user can result in broken permissions and conflicting behavior with the system package manager. It is recommended to use a virtual environment instead: https://pip.pypa.io/warnings/venv_. This warning is often seen when installing packages on the cloud instance, but can be ignored.

## Running Examples
We have used FogROS for 3 example use-cases (motion planning, grasp planning, and SLAM map building). Please see our [examples repo](https://github.com/BerkeleyAutomation/fogros2-examples) for these and how to run them.


## References
<a id="1">[1]</a> 
Macenski Steven, Foote Tully, Gerkey Brian, Lalancette Chris, and Woodall William, 
“Robot Operating System 2: Design, architecture, and uses in the wild,” 
Science Robotics, vol. 7, no. 66, p. eabm6074, doi: 10.1126/scirobotics.abm6074.

