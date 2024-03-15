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

    # step 1: your service (cloud) node 
    service_node = Node(
        package="bench",
        executable="add_three_ints_service",
    )

    sgc_router = Node(
        package="sgc_launch",
        executable="sgc_router",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"config_file_name": "service-client.yaml"}, # step 2: your yaml file name 
            {"whoami": "machine_server"},
            {"release_mode": True},
        ],
    )

    fogros2.SkyLaunchDescription(
        nodes=[service_node, sgc_router],
        mode="spot",  # launch, benchmark, spot
        # ami="ami-0f43c97344dd92658", # default parameter is a ubuntu 22.04 image
    )

    return LaunchDescription(
        [
            # step 3: your client node
            Node(
                package="bench",
                executable="add_three_ints_client",
            ),
            Node(
                package="sgc_launch",
                executable="sgc_router",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"config_file_name": "service-client.yaml"}, # step 4: your yaml file name
                    {"whoami": "machine_client"},
                    {"release_mode": True},
                ],
            ),
        ]
    )
