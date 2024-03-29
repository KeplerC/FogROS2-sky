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

    fogros2.SkyLaunchDescription(
        workdir = "", 
        containers = [
                "sudo docker run -d --net=host -v ~/.sky:/root/.sky -v ~/sky_benchmark_dir:/root/sky_benchmark_dir --rm keplerc/fogros-sky-latency:latest ros2 run fogros2 latency --ros-args -p request_topic_name:=/robot_mesh -p request_topic_type:=std_msgs/msg/UInt8MultiArray -p response_topic_name:=/motion_plan -p response_topic_type:=std_msgs/msg/Float64MultiArray  -p heuristic_mode:=first_request",
                "sudo docker run -d --net=host -ti keplerc/mpt:latest ros2 run mpt_ros node",
                "sudo docker run -d --net=host -ti keplerc/mpt:latest ros2 run mpt_ros client"
                ],
        mode = "benchmark", # launch, benchmark
    )

    # this is to prevent the launch description from exiting 
    # TODO: a better way 
    return LaunchDescription([
        Node(
            package="fogros2_examples", executable="listener", output="screen", # listener
        ), 
    ])
