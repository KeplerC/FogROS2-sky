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
from .sky_cluster_manager import SkyCluster, SkySpotCluster
from .sky_yaml_builder import SkyYamlBuilder

resource_str = '''
resources:
    cloud: aws 
    disk_size: 128
    region: us-west-1
    image_id: ami-0ce2cb35386fc22e9
'''

benchmark_resource_str = '''
resources:
    cloud: aws 
    candidates:
    - {cpus: 8}
'''
class SkyLaunchDescription():
    def __init__(self, 
                 workdir = "~/sky_ws/src",
                 nodes = [], 
                 containers = [],
                 mode = "launch", 
                 ami = "ami-0ce2cb35386fc22e9", 
                 additional_setup_commands = [],
                 additional_run_commands = []):
        self.logger = logging.get_logger(__name__)
        
        self.mode = mode

        self.nodes = nodes
        if self.nodes:
            self._generate_to_cloud_nodes()

        self.yaml_builder = SkyYamlBuilder(
            workdir=workdir, 
            docker_cmd=containers,
            ami = ami, 
            additional_setup_commands = additional_setup_commands,
            additional_run_commands = additional_run_commands
            )

        config_path = "/tmp/sky.yaml"
        self.yaml_builder.output_yaml_config(config_path)

        if self.mode == "launch":
            self.cluster = SkyCluster(config_path, self.logger)

            self.cluster.init_cluster()
        elif self.mode == "benchmark":
            self.cluster = SkyCluster(config_path, self.logger)
            thread = Thread(target=self.benchmark, args=[])
            thread.start()
        elif self.mode == "spot":
            self.cluster = SkySpotCluster(config_path, self.logger)
            self.cluster.init_cluster()
        else:
            pass

    def _generate_to_cloud_nodes(self):
        with open(f"/tmp/to_cloud", "wb+") as f:
            self.logger.info(f"dumping {self.nodes} to /tmp/to_cloud")
            dumped_node_str = pickle.dumps(self.nodes)
            f.write(dumped_node_str)
        
        # self.cluster.init_cluster()

    def benchmark(self):
        # TODO: benchmark the scheduler
        pass


