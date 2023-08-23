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
    pip3 install colcon-common-extensions > /dev/null 2>&1
    # install cloud dependencies 
    echo Installing other dependencies...
    sudo apt-get install -y python3-pip unzip python3-pip ros-humble-rmw-cyclonedds-cpp #> /dev/null 2>&1
    pip3 install boto3 paramiko scp wgconfig sky #> /dev/null 2>&1
    pip3 install pyopenssl --upgrade # > /dev/null 2>&1
    # install sky callback
    pip install "git+https://github.com/skypilot-org/skypilot.git#egg=sky-callback&subdirectory=sky/callbacks/" > /dev/null 2>&1
    mkdir -p ~/fog_ws
    ln -s ~/sky_workdir/src ~/fog_ws/src
    ln -s ~/sky_workdir/install ~/fog_ws/install
    echo cloud dependencies installed.
    source ~/fog_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py 

file_mounts:
    /tmp/to_cloud_nodes : /tmp/to_cloud
"""

execute_command_ros_node = """
run: |
    echo "hello"
    source ~/fog_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py 
"""

setup_command_docker = """
setup: |
    sudo apt-get install -y docker.io
    sudo systemctl reset-failed docker
    sleep 1
    sudo systemctl start docker
"""

def get_sky_config_yaml(
    workdir,
    docker_cmd = []
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
    else:
        config += setup_command_ros_node + "\n"
        config += execute_command_ros_node + "\n"
    
    config += """
resources:
    disk_size: 128
    cloud: aws 
    region: us-west-1
    image_id: ami-03c44768198d7e3fe
""" 
    return config 


# get_sky_config_yaml(
#     workdir="~/sky_ws/", 
#     docker_cmd=[
#     "sudo docker run -d --net=host -v --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros client.launch.py",
#     "sudo docker run --net=host -v ~/.sky:/root/.sky -v ~/sky_benchmark_dir:/root/sky_benchmark_dir --rm keplerc/gqcnn_ros:skybench ros2 launch gqcnn_ros planner.launch.py"]
# )