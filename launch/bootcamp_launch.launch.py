# Copyright 2021 the Autoware Foundation
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
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
"""
Launch file for vehicle for Autoware bootcamp (Lincoln MKZ @Pennovation).
"""

import os
from ament_index_python import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
from launch import LaunchDescription


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(get_package_share_directory(package_name),
                             'config', file_name)
    return launch.substitutions.LaunchConfiguration('params',
                                                    default=[file_path])


def generate_launch_description():
    """Generate launch description with a single component."""
    dataspeed_ford_dbw = Node(
        executable='dbw_node',
        name='dataspeed_ford_dbw_node',
        namespace='vehicle',
        package='dbw_ford_can',
        parameters=[get_param_file('bootcamp_launch', 'dbw_params.yaml')])

    return LaunchDescription([dataspeed_ford_dbw])
