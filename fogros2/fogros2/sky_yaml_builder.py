setup_command_ros_node = """
setup: |
    # need to deactivate conda to install in system Python env
    conda deactivate
    # install ROS
    echo Installing ROS... It takes a while... 
    sudo apt-get update > /dev/null 2>&1
    sudo apt-get install -y software-properties-common gnupg lsb-release > /dev/null 2>&1
    sudo add-apt-repository universe > /dev/null 2>&1
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update > /dev/null 2>&1
    sudo apt-get install -y ros-humble-desktop > /dev/null 2>&1
    # install cloud dependencies 
    echo Installing other dependencies...
    sudo apt-get install -y python3-pip unzip python3-pip ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions
    pip3 install boto3 paramiko scp skypilot-nightly[aws] #> /dev/null 2>&1
    pip3 install pyopenssl --upgrade # > /dev/null 2>&1
    # install sky callback
    # pip install "git+https://github.com/skypilot-org/skypilot.git#egg=sky-callback&subdirectory=sky/callbacks/" > /dev/null 2>&1
    mkdir -p ~/fog_ws
    ln -s ~/sky_workdir ~/fog_ws/src
    echo cloud dependencies installed.
    echo Building ROS2 workspace...
    source /opt/ros/humble/setup.bash && cd ~/fog_ws && colcon build 
    cp -r /tmp/crypto ~/fog_ws/install/sgc_launch/share/sgc_launch/configs/crypto/
    echo Install SGC dependencies 
    sudo apt install -y build-essential curl pkg-config libssl-dev protobuf-compiler clang
    curl https://sh.rustup.rs -sSf | sh -s -- -y && source "$HOME/.cargo/env"

file_mounts:
    /tmp/to_cloud_nodes : /tmp/to_cloud
    # TODO: make it as code
    /tmp/crypto : ~/sky_ws/install/sgc_launch/share/sgc_launch/configs/crypto
"""

execute_command_ros_node = """
run: |
    echo "hello"
    source ~/fog_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py 
"""

setup_command_docker = """
setup: |
    # sudo apt-get update && sudo apt-get install -y docker.io
    # sudo systemctl reset-failed docker
    # sleep 20
    # sudo systemctl start docker
"""

def get_sky_config_yaml(
    workdir,
    docker_cmd = [],
    resource_str = "",
    benchmark_resource_str = "",
):
    config = """
name: fogros2-sky-cluster
num_nodes: 1  # Number of VMs to launch
"""
    if workdir:
        config += "\nworkdir: " + workdir + "\n"
    
    if docker_cmd:
        config += setup_command_docker + "\n"
        config += "run: |\n"
        for cmd in docker_cmd:
            config += "    " + cmd + "\n"
    else: #TODO: need to handle the cast that both nodes and containers exist
        config += setup_command_ros_node + "\n"
        config += execute_command_ros_node + "\n"
    
    config += resource_str
    config += benchmark_resource_str
    return config 


# get_sky_config_yaml(
#     workdir="~/sky_ws/", 
#     docker_cmd=[
#     "sudo docker run -d --net=host -v --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros client.launch.py",
#     "sudo docker run --net=host -v ~/.sky:/root/.sky -v ~/sky_benchmark_dir:/root/sky_benchmark_dir --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros planner.launch.py"]
# )