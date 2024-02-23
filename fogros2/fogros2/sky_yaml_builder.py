import os
import yaml

DOCKER_SETUP_CMD = """
# sudo apt-get update && sudo apt-get install -y docker.io
# sudo systemctl reset-failed docker
# sleep 20
# sudo systemctl start docker
"""

SETUP_ENV_CMD = """
# need to deactivate conda to install in system Python env
conda deactivate
# install ROS
echo Installing ROS... It takes a while... 
sudo apt-get update > /dev/null 2>&1
sudo apt-get install -y software-properties-common gnupg lsb-release docker.io > /dev/null 2>&1
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


class SkyYamlBuilder:
    def __init__(
        self,
        workdir,
        num_replica=1,
        job_name="fogros2-sky-spot",
        docker_cmd=[],
        resource_str="",
        benchmark_resource_str="",
        cloud="aws",
        disk_size=128,
        region="us-west-1",
        ami="ami-0ce2cb35386fc22e9",
        additional_setup_commands=[],
        additional_run_commands=[],
        accelerator="",
        cpus="",
    ):
        self.workdir = workdir
        self.docker_cmd = docker_cmd
        self.resource_str = resource_str
        self.benchmark_resource_str = benchmark_resource_str
        self.workspace_path = os.getenv("COLCON_PREFIX_PATH", "")
        self.config = {
            "name": job_name,
            "num_nodes": num_replica,
            "workdir": self.workdir,
            "setup": self.get_setup_command(additional_setup_commands),
            "run": self.get_execution_command(additional_run_commands),
            "file_mounts": self.get_file_mount_command(),
            "resources": self.get_resources(
                cloud=cloud,
                disk_size=disk_size,
                region=region,
                ami=ami,
                accelerator=accelerator,
                cpus=cpus,
            ),
        }

    def get_file_mount_command(self):
        crypto_path = os.path.join(
            self.workspace_path, "sgc_launch/share/sgc_launch/configs/crypto"
        )
        file_mounts = {
            "/tmp/to_cloud_nodes": "/tmp/to_cloud",
            "/tmp/crypto": crypto_path,
        }
        return file_mounts

    def get_setup_command(self, additional_setup_commands):
        setup_cmd = ""
        if self.docker_cmd:
            setup_cmd = DOCKER_SETUP_CMD
        else:
            setup_cmd = SETUP_ENV_CMD
        setup_cmd += "\n".join(["    " + cmd for cmd in additional_setup_commands])
        return setup_cmd

    def get_execution_command(self, additional_run_commands):
        run_command = ""
        if additional_run_commands:
            run_command += "\n".join(["    " + cmd for cmd in additional_run_commands])

        if self.docker_cmd:
            run_command += "\n".join(["    " + cmd for cmd in self.docker_cmd])
        else:
            # sgc_docker_cmd = '''sudo docker run -d --net=host -it keplerc/fogros2-rt-router:latest bash -c ". ./install/setup.sh && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run sgc_launch sgc_router --ros-args -p config_file_name:=service-client.yaml -p whoami:=machine_server -p release_mode:=True"'''
            cloud_cmd = "source ~/fog_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 launch fogros2 cloud.launch.py"
            run_command += cloud_cmd
        return run_command

    def get_resources(
        self,
        cloud,
        disk_size,
        region,
        ami,
        accelerator="",
        cpus = "",
    ):
        resource = {}
        resource["cloud"] = cloud
        resource["disk_size"] = disk_size
        resource["region"] = region
        resource["image_id"] = ami
        if accelerator:
            resource["accelerators"] = accelerator
        if cpus:
            resource["cpus"] = cpus
        return resource

    def output_yaml_config(self, config_path):
        print(self.config)
        with open(config_path, "w") as file:
            yaml.dump(self.config, file, sort_keys=False, default_style="|")


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
