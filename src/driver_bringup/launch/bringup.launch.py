import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

sys.path.append(os.path.join(get_package_share_directory('driver_bringup'),'launch'))

def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('driver_bringup'), 'config', 'launch_params.yaml')))
    
    def get_params(name):
        return os.path.join(get_package_share_directory('driver_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))
    
    #camera
    if launch_params['use_video']: 
        image_node  = ComposableNode(
            package='camera_driver',
            plugin='camera_driver::VideoDriverNode',
            name='video_driver',
            parameters=[get_params('video_driver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    else:
        # TODO
        pass

    #serial
    if launch_params['use_virtual_serial']:
        serial_driver_node = Node(
            package='auto_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[get_params('virtual_serial')],
            ros_arguments=['--ros-args',],
        )
    else:
        serial_driver_node = Node(
            package='auto_serial_driver',
            executable='serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('serial_driver')],
            ros_arguments=['--ros-args',],
        )

    # 使用intra cmmunication提高图像的传输速度
    def get_camera_detector_container(*detector_nodes):
        nodes_list = list(detector_nodes)
        nodes_list.append(image_node)
        container = ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes_list,
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        return TimerAction(
            period=2.0,
            actions=[container],
        )
    
    # cpp detect
    cpp_detector_node = ComposableNode(
        package='detector_cpp', 
        plugin='driver_detector::DetectorNode',
        name='detector_cpp',
        parameters=[get_params('detector_cpp'),{'image_encodings': 'rgb8' if not launch_params['use_video'] else 'bgr8'}],
        extra_arguments=[{'use_intra_process_comms': True}],
        
    )

    cam_detector_node=get_camera_detector_container(cpp_detector_node)

    #web
    web_node  = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='web_node',
            output='both',
            emulate_tty=True,
            #parameters=[get_params('web_node')],
            ros_arguments=['--ros-args','-p','port:=9090'],
        )
    
    web_video_node=Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server_node',
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args','-p','address:=0.0.0.0'],
        )

    #delay to start
    delay_serial_node = TimerAction(
        period=0.5,
        actions=[serial_driver_node],
    )
    delay_web_node = TimerAction(
        period=0.8,
        actions=[web_node],
    )
    delay_web_video_node=TimerAction(
        period=1.0,
        actions=[web_video_node],
    )

    push_namespace = PushRosNamespace(launch_params['namespace'])
    
    launch_description_list = [
        push_namespace,
        cam_detector_node,
        delay_web_node,
        delay_web_video_node,
        delay_serial_node]
    
    return LaunchDescription(launch_description_list)