import os 


EXECUTION_CMD = """
run: |
    source ~/fog_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py 
"""

DOCKER_SETUP_CMD = """
setup: |
    # sudo apt-get update && sudo apt-get install -y docker.io
    # sudo systemctl reset-failed docker
    # sleep 20
    # sudo systemctl start docker
"""

SETUP_ENV_CMD = """
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
"""

FILE_MOUNT_CMD = """
file_mounts:
    /tmp/to_cloud_nodes : /tmp/to_cloud
    # TODO: make it as code
    /tmp/crypto : {}
"""

class SkyYamlBuilder:
    def __init__(
        self, 
        workdir, 
        docker_cmd=[], 
        resource_str="", 
        benchmark_resource_str=""
    ):
        self.workdir = workdir
        self.docker_cmd = docker_cmd
        self.resource_str = resource_str
        self.benchmark_resource_str = benchmark_resource_str

        self.workspace_path = os.environ["COLCON_PREFIX_PATH"]

        self.config = {}


    def get_file_mount_command(self):
        crypto_path = os.path.join(self.workspace_path, "sgc_launch/share/sgc_launch/configs/crypto")
        return FILE_MOUNT_CMD.format(
            crypto_path
        )
    
    def get_setup_command(self):
        if self.docker_cmd:
            return DOCKER_SETUP_CMD
        else:
            return SETUP_ENV_CMD
        
    def get_execution_command(self):
        if self.docker_cmd:
            config += "run: |\n"
            for cmd in self.docker_cmd:
                config += "    " + cmd + "\n"
            return config
        else:
            return EXECUTION_CMD

    def output_yaml_config(self, config_path):
        pass 
    
# def get_sky_config_yaml(
#     workdir,
#     docker_cmd=[],
#     resource_str="",
#     benchmark_resource_str="",
# ):
#     config = """
# name: fogros2-sky-cluster
# num_nodes: 1  # Number of VMs to launch
# """
#     if workdir:
#         config += "\nworkdir: " + workdir + "\n"

# if docker_cmd:
    
# else:  # TODO: need to handle the cast that both nodes and containers exist
#     config += setup_command_ros_node + "\n"
#     config += execute_command_ros_node + "\n"

# config += resource_str
# config += benchmark_resource_str
# return config


# get_sky_config_yaml(
#     workdir="~/sky_ws/",
#     docker_cmd=[
#     "sudo docker run -d --net=host -v --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros client.launch.py",
#     "sudo docker run --net=host -v ~/.sky:/root/.sky -v ~/sky_benchmark_dir:/root/sky_benchmark_dir --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros planner.launch.py"]
# )
