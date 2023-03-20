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
from threading import Thread
from time import sleep

from .command_builder import BashBuilder
from .name_generator import get_unique_name
from .cloud_instance import CloudInstance
import sky

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

    def launch_cloud_node(self):
        # override the base class method
        # launch cloud node with SGC network instead of VPN
        cmd_builder = BashBuilder()
        # cmd_builder.append(f"source /opt/ros/{self.ros_distro}/setup.bash")
        # cmd_builder.append(
        #     f"cd /home/{self._username}/fog_ws && /home/{self._username}/.local/bin/colcon build --cmake-clean-cache"
        # )
        cmd_builder.append(f"source /home/{self._username}/fog_ws/install/setup.bash")
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
        if not ros_domain_id:
            ros_domain_id = 0
        cmd_builder.append(
            f"ROS_DOMAIN_ID={ros_domain_id} "
            "ros2 launch fogros2 cloud.launch.py"
        )
        self.logger.info(cmd_builder.get())
        self.scp.execute_cmd(cmd_builder.get())
    
    def run_sgc(self):
        # restart docker deamon
        # ps axf | grep docker | grep -v grep | awk '{print "kill -9 " $1}' | sudo sh 
        # sudo systemctl start docker
        
        # self.scp.execute_cmd("sudo dockerd --debug")
        #self.scp.execute_cmd("systemctl status docker.service")
        self.scp.execute_cmd("sudo systemctl reset-failed docker")
        self.scp.execute_cmd("sudo systemctl start docker")
        sleep(1)
        self.scp.execute_cmd("systemctl status docker.service")
        self.scp.execute_cmd("docker run -d --net=host -e GATEWAY_IP=128.32.37.48 keplerc/fogros2-sgc:v0.1 bash -c 'source /opt/ros/humble/setup.bash && /gdp-router router'")
    def create(self, config):
        # create a new Sky cluster
        # param config: sky config file
        self.logger.info(f"Creating new Sky cluster {self._name}")
        self.create_sky_instance(config)
        self.info(flush_to_disk=True)
        # self.connect()
        self._is_created = True
        # self.run_sgc()
        # def run_cloud_cmd_fn():
        #     self.launch_cloud_node()
        # thread = Thread(target=run_cloud_cmd_fn, args=[])
        # thread.start()
    
    def create_sky_instance(self, sky_yaml_config):
        '''Create Sky Instance with skypilot'''
        user = subprocess.check_output('whoami', shell=True).decode().strip()

        with open("/tmp/sky.yaml", "w+") as f:
            f.write(sky_yaml_config)

        with sky.Dag() as dag:
            t = sky.Task.from_yaml("/tmp/sky.yaml")

        if sky.status("sky-fogros") == []:
            # create a new cluster
            # sky.launch(dag, cluster_name = "sky-fogros", idle_minutes_to_autostop=100)
            pid = os.fork()
            if pid == 0:
                # child process, just launch the yaml cloud node
                sky.launch(dag, cluster_name = "sky-fogros", idle_minutes_to_autostop=100)
                exit(0)
        else:
            # run with the same cluster
            sky.exec(dag, cluster_name = "sky-fogros")


        while (sky.status("sky-fogros") == []):
            sleep(1)
        status = sky.status("sky-fogros")[0]
        # here we only need the ip address of the head node
        try:
            self._ip = status["handle"].__dict__["stable_internal_external_ips"][0][1] 
            self._ssh_key_path = f"/home/{user}/.ssh/sky-key"
        except:
            # TODO: placeholders
            self._ip = None
            self._ssh_key_path = f"/home/{user}/.ssh/sky-key"
        self._is_created = True