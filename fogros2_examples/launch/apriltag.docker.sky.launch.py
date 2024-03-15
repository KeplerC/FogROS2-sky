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

from launch import LaunchDescription
from launch_ros.actions import Node

import fogros2


def generate_launch_description():
    """Talker example that launches everything locally."""


    # method 2: use dockerized sgc 
    fogros2.SkyLaunchDescription(
        nodes=[],
        mode="spot",  # launch, benchmark, spot
        # ami="ami-0f43c97344dd92658", # default parameter is a ubuntu 22.04 image
        additional_setup_commands = [],
        additional_run_commands = [
            "sudo apt-get update", 
            "sudo apt-get install -y docker.io",
            "sudo docker run -d --net=host keplerc/apriltag:service bash -c \"source install/setup.bash && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch apriltag_ros_fork srv_server.launch.py\"",
            "sudo docker run --net=host keplerc/fogros2-rt-router:latest bash -c \"echo 'hello'>install/sgc_launch/share/sgc_launch/configs/crypto/test_cert/test_cert-private.pem  &&  source ./install/setup.sh && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run sgc_launch sgc_router --ros-args -p config_file_name:=service-apriltag.yaml -p whoami:=machine_server -p release_mode:=False\"",
            ],
        num_replica = 1,
        skip_setup = True,
    ) # gpu option available (see SAM example)

    fogros2.SkyLaunchDescription(
        nodes=[],
        mode="spot",  # launch, benchmark, spot
        # ami="ami-0f43c97344dd92658", # default parameter is a ubuntu 22.04 image
        additional_setup_commands = [],
        additional_run_commands = [
            "sudo apt-get update", 
            "sudo apt-get install -y docker.io",
            "sudo docker run -d --net=host keplerc/apriltag:service bash -c \"source install/setup.bash && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch apriltag_ros_fork srv_server.launch.py\"",
            "sudo docker run --net=host keplerc/fogros2-rt-router:latest bash -c \"echo 'hello'>install/sgc_launch/share/sgc_launch/configs/crypto/test_cert/test_cert-private.pem  &&  source ./install/setup.sh && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run sgc_launch sgc_router --ros-args -p config_file_name:=service-apriltag.yaml -p whoami:=machine_server -p release_mode:=False\"",
            ],
        num_replica = 1,
        skip_setup = True,
    ) # gpu option available (see SAM example)

    return LaunchDescription(
        [
        ]
    )
