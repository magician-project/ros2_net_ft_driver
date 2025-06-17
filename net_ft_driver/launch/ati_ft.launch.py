# Copyright 2025 HHCM, Istituto Italiano di Tecnologia
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    # External args
    sensor_ip_arg = DeclareLaunchArgument("sensor_ip", default_value="192.168.1.1")
    sensor_filter_value_arg = DeclareLaunchArgument("sensor_filter_value", default_value="0")
    sensor_sampling_rate_arg = DeclareLaunchArgument("sensor_sampling_rate", default_value="500")
    wrench_topic_name_arg = DeclareLaunchArgument("wrench_topic_name", default_value="/ati_ft_sensor/wrench_sensed")
    diagnostic_topic_name_arg = DeclareLaunchArgument("diagnostic_topic_name", default_value="/ati_ft_sensor/diagnostic")
    reset_bias_service_name_arg = DeclareLaunchArgument("reset_bias_service_name", default_value="/ati_ft_sensor/reset_bias")
    set_filter_service_name_arg = DeclareLaunchArgument("set_filter_service_name", default_value="/ati_ft_sensor/set_filter")
    set_sampling_rate_service_name_arg = DeclareLaunchArgument("set_sampling_rate_service_name", default_value="/ati_ft_sensor/set_sampling_rate")
    node_rate_arg = DeclareLaunchArgument("node_rate", default_value="500.0", description="This should be equal or bigger than the sensor_sampling_rate to not lose packets and to not receive the recent values very late (there seems to be a FIFO buffer)")
    sensing_frame_arg = DeclareLaunchArgument("sensing_frame", default_value="ft_sensing_frame", description="To fill the header in the wrench message. So set this according to the name of the sensing frame in the urdf, taking care that is positioned correctly")

    ati_ft_sensor_node = Node(
        package="net_ft_driver",
        executable="ati_ft_main",
        name="ati_ft_main",
        output="screen",
        parameters=[
            {"sensor_ip": LaunchConfiguration("sensor_ip")},
            {"sensor_filter_value": LaunchConfiguration("sensor_filter_value")},
            {"sensor_sampling_rate": LaunchConfiguration("sensor_sampling_rate")},
            {"wrench_topic_name": LaunchConfiguration("wrench_topic_name")},
            {"diagnostic_topic_name": LaunchConfiguration("diagnostic_topic_name")},
            {"reset_bias_service_name": LaunchConfiguration("reset_bias_service_name")},
            {"set_filter_service_name": LaunchConfiguration("set_filter_service_name")},
            {"set_sampling_rate_service_name": LaunchConfiguration("set_sampling_rate_service_name")},
            {"node_rate": LaunchConfiguration("node_rate")},
            {"sensing_frame": LaunchConfiguration("sensing_frame")},
        ],
    )


    return LaunchDescription(
        [
            sensor_ip_arg,
            sensor_filter_value_arg,
            sensor_sampling_rate_arg,
            wrench_topic_name_arg,
            diagnostic_topic_name_arg,
            reset_bias_service_name_arg,
            set_filter_service_name_arg,
            set_sampling_rate_service_name_arg,
            node_rate_arg,
            sensing_frame_arg,
            ati_ft_sensor_node,
        ]
    )