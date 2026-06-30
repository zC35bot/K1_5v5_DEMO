# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package ='game_controller',
            executable='game_controller_node',
            name='game_controller',
            output='screen',
            parameters=[
                {
                    # 监听的端口，默认为 GameController 广播端口 3838
                    "port": 3838,

                    # 是否开启 IP 白名单，开启后会忽略非白名单 IP 发的广播消息，默认不应该开启
                    # 默认关闭白名单，与 game_controller_node.cpp:16 的 C++ 默认(false)及上方注释一致，
                    # 避免主机 IP 不符时静默丢弃全部裁判机报文；如需启用请按需配置正确的 ip_white_list
                    "enable_ip_white_list": False,

                    # 只接收指定
                    "ip_white_list": [
                        "192.168.74.2",
                    ],
                }
            ]
        ),
    ])
