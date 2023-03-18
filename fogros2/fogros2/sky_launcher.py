# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
import pickle
from .sky_instance import SkyInstance


def get_sky_config_yaml():
    config = """
name: fogros2-sky-cluster

resources:
    disk_size: 45
    cloud: aws 

num_nodes: 1  # Number of VMs to launch

# Working directory (optional) containing the project codebase.
# Its contents are synced to ~/sky_workdir/ on the cluster.
workdir: ~/fog_ws/install

file_mounts:
    /tmp/to_cloud_nodes : /tmp/to_cloud
# Commands to be run before executing the job.
# Typical use: pip install -r requirements.txt, git clone, etc.
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
    sudo apt-get install -y ros-rolling-desktop > /dev/null 2>&1
    pip3 install colcon-common-extensions > /dev/null 2>&1
    # install cloud dependencies 
    echo Installing other dependencies...
    sudo apt-get install -y python3-pip unzip docker.io python3-pip ros-rolling-rmw-cyclonedds-cpp > /dev/null 2>&1
    pip3 install boto3 paramiko scp wgconfig sky > /dev/null 2>&1
    pip3 install pyopenssl --upgrade > /dev/null 2>&1
    mkdir ~/fog_ws
    ln -s ~/sky_workdir ~/fog_ws/install
    echo cloud dependencies installed.
# Commands to run as a job.
# Typical use: launch the main program.
run: |
    # for some reason, I need to restart the docker 
    # ps axf | grep docker | grep -v grep | awk '{print "kill -9 " $1}' | sudo sh 
    # sudo systemctl start docker
    # run SGC 
    # docker run -d --net=host -e GATEWAY_IP=128.32.37.48 keplerc/fogros2-sgc:v0.1 bash -c "source /opt/ros/humble/setup.bash && /gdp-router router"
    # source /opt/ros/rolling/setup.bash && cd /home/ubuntu/fog_ws && colcon build --cmake-clean-cache &&  source ~/fog_ws/install/setup.bash && ROS_DOMAIN_ID=0 ros2 launch fogros2 cloud.launch.py &
""" 
    return config
# docker run --net=host keplerc/fogros2-sgc:v0.1 bash -c "source /opt/ros/humble/setup.bash && /gdp-router router"
class SkyLauncher():
    def __init__(self) -> None:
        self.nodes = []
        self.cluster = SkyCluster()

    def add(self, nodes):
        # add nodes to the scheduler
        # nodes is a list of SkyNode
        self.nodes += nodes
        print(self.nodes)

    def run(self):
        # run the scheduler
        with open(f"/tmp/to_cloud", "wb+") as f:
            print(f"to be dumped")
            dumped_node_str = pickle.dumps(self.nodes)
            f.write(dumped_node_str)
        
        self.cluster.init_cluster()

    def benchmark(self):
        # TODO: benchmark the scheduler
        pass

class SkyCluster():
    def __init__(self) -> None:
        self.sky_instance = SkyInstance()

    def init_cluster(self):
        sky_yaml_config = get_sky_config_yaml()
        self.sky_instance.create(sky_yaml_config)
