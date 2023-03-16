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
# licensing opportunities. IN NO EVEpNT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import json
import os
import subprocess
from .name_generator import get_unique_name
from .cloud_instance import CloudInstance
import sky

sky_yaml_config = """
name: fogros2-sky-cluster

resources:
    disk_size: 45

num_nodes: 1  # Number of VMs to launch

# Working directory (optional) containing the project codebase.
# Its contents are synced to ~/sky_workdir/ on the cluster.
workdir: ~/fog_ws

# Commands to be run before executing the job.
# Typical use: pip install -r requirements.txt, git clone, etc.
setup: |
    # install ROS
    conda deactivate
    sudo apt-get update
    sudo apt-get install -y software-properties-common gnupg lsb-release 
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-rolling-desktop
    pip3 install colcon-common-extensions
    # install cloud dependencies 
    sudo apt-get install -y python3-pip wireguard unzip docker.io python3-pip ros-rolling-rmw-cyclonedds-cpp
    pip3 install boto3 paramiko scp wgconfig sky
    pip3 install pyopenssl --upgrade
    ln -s ~/sky_workdir ~/fog_ws
# Commands to run as a job.
# Typical use: launch the main program.
run: |
    conda env list
    source /opt/ros/rolling/setup.bash && cd /home/ubuntu/fog_ws && colcon build --cmake-clean-cache && . /home/ubuntu/fog_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml && ROS_DOMAIN_ID=0 ros2 launch fogros2 cloud.launch.py
"""

class SkyInstance(CloudInstance):
    """Sky Interface of CloudInstance."""

    def __init__(
            self,
            **kwargs,
    ):
        super().__init__(**kwargs)
        self.cloud_service_provider = "SKY"
        self._name = get_unique_name()

        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)

        # after config
        self._ssh_key = None
        self.zone = "sky"
        self.type = "sky"
        self.compute_instance_disk_size = "sky"

        self.create()

    def create(self):
        self.logger.info(f"Creating new Sky cluster {self._name}")
        self.create_sky_instance()
        self.info(flush_to_disk=True)
        self.connect()
        self.install_colcon()
        self.install_cloud_dependencies()
        self.info(flush_to_disk=True)
        self._is_created = True

    def info(self, flush_to_disk=True):
        info_dict = super().info(flush_to_disk)
        info_dict["compute_region"] = self.zone
        info_dict["compute_instance_type"] = self.type
        info_dict["disk_size"] = self.compute_instance_disk_size
        info_dict["compute_instance_id"] = self._name
        if flush_to_disk:
            with open(os.path.join(self._working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict
    
    def create_sky_instance(self):
        '''Create Sky Instance with skypilot'''
        user = subprocess.check_output('whoami', shell=True).decode().strip()

        with open("/tmp/sky.yaml", "w+") as f:
            f.write(sky_yaml_config)

        # with sky.Dag() as dag:
        #     t = sky.Task.from_yaml("/tmp/sky.yaml")

        # sky.launch(dag, cluster_name = "sky-gdpmobile8", idle_minutes_to_autostop=100)

        status = sky.status("sky-gdpmobile8")[0]
        # here we only need the ip address of the head node
        self._ip = status["handle"].__dict__["stable_internal_external_ips"][0][1] 
        self._ssh_key_path = f"/home/{user}/.ssh/sky-key"
        self._is_created = True