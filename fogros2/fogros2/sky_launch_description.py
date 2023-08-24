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
from threading import Thread
from rclpy.node import Node
from rclpy import logging
from .sky_yaml_builder import get_sky_config_yaml

resource_str = '''
resources:
    cloud: aws 
    disk_size: 128
    region: us-west-1
    image_id: ami-03c44768198d7e3fe
'''

benchmark_resource_str = '''
resources:
    cloud: aws 
    candidates:
    - {cpus: 8}
'''
class SkyLaunchDescription():
    def __init__(self, 
                 workdir = "~/sky_ws/",
                 nodes = [], 
                 containers = [],
                 mode = "launch"):
        self.logger = logging.get_logger(__name__)
        
        self.mode = mode

        self.nodes = nodes
        if self.nodes:
            self._generate_to_cloud_nodes()

        if self.mode == "launch":
            self.cluster = SkyCluster(get_sky_config_yaml(
                    workdir=workdir, 
                    docker_cmd=containers,
                    resource_str = resource_str,
            ), self.logger)

            thread = Thread(target=self.launch, args=[])
            thread.start()
        elif self.mode == "benchmark":
            self.cluster = SkyCluster(get_sky_config_yaml(
                    workdir=workdir, 
                    docker_cmd=containers,
                    benchmark_resource_str=benchmark_resource_str,
            ), self.logger)
            thread = Thread(target=self.benchmark, args=[])
            thread.start()
        else:
            pass

    def _generate_to_cloud_nodes(self):
        with open(f"/tmp/to_cloud", "wb+") as f:
            self.logger.info(f"dumping {self.nodes} to /tmp/to_cloud")
            dumped_node_str = pickle.dumps(self.nodes)
            f.write(dumped_node_str)
        
        # self.cluster.init_cluster()

    def launch(self):
        #TODO: create a separate thread for it
        self.logger.info(f"launching the Sky cluster")
        self.cluster.init_cluster()

    def benchmark(self):
        # TODO: benchmark the scheduler
        pass



# TODO: later separate to another file
import sky 
import subprocess
from time import sleep
import os 
from .name_generator import get_unique_name
class SkyCluster():
    def __init__(self, sky_yaml_config, logger) -> None:
        self.sky_yaml_config = sky_yaml_config
        self.logger = logger
        self._name = get_unique_name()
        with open("/tmp/sky.yaml", "w+") as f:
            f.write(sky_yaml_config)
        self.logger.info(f"Creating new Sky cluster {self._name} with config: {self.sky_yaml_config}")

    def init_cluster(self):
        self.logger.info(f"Creating new Sky cluster {self._name}")
        self.create_sky_instance(self.sky_yaml_config)
        self._is_created = True

    def create_sky_instance(self):
        '''Create Sky Instance with skypilot'''
        user = subprocess.check_output('whoami', shell=True).decode().strip()

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