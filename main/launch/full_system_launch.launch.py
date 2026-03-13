# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    full_system_launch.launch.py                       :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: rortiz <rortiz@student.42madrid.com>       +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2026/03/13 04:23:07 by rortiz            #+#    #+#              #
#    Updated: 2026/03/13 04:25:07 by rortiz           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "image_capturer",
            executable = 
        )
    ])