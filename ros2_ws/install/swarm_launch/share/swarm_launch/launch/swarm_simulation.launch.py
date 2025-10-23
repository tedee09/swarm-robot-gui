from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os, shutil, sys

def generate_launch_description():
    robot_ids = [1, 2, 3, 4]

    # Resolve Webots path secara dinamis
    webots_exec = shutil.which("webots")
    if webots_exec is None:
        raise RuntimeError("Webots tidak ditemukan. Pastikan tersedia di PATH.")

    world_path = os.path.expanduser('~/swarm/webots/worlds/swarm.wbt')
    rviz_path = os.path.expanduser('~/swarm/ros2_ws/src/rviz2/swarm.rviz')
    assert os.path.exists(world_path), f"World tidak ditemukan: {world_path}"
    assert os.path.exists(rviz_path),  f"RViz config tidak ditemukan: {rviz_path}"

    launch_nodes = []

    # Jalankan Webots
    launch_nodes.append(
        ExecuteProcess(
            cmd=[webots_exec, world_path],
            output='screen',
            additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
        )
    )

    # Jalankan setiap robot node sesuai ID-nya
    for rid in robot_ids:
        
        launch_nodes += [
            Node(
                package='path_planner',
                executable='path_planner_node',
                name=f'path_planner_{rid}',
                namespace=f'robot{rid}',
                parameters=[{'robot_id': rid}],
                output='screen'
            ), 
            Node(
                package='path_executor',
                executable='path_executor_node',
                name=f'path_executor_{rid}',
                namespace=f'robot{rid}',
                parameters=[{'robot_id': rid}],
                output='screen'
            ),
        ]

    # Vision Node (satu untuk semua robot)
    launch_nodes.append(
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            parameters=[{'robot_ids': robot_ids}],
            output='screen'
        )
    )

    # Role Manager Node (satu untuk semua robot)
    launch_nodes.append(
        Node(
            package='role_manager',
            executable='role_manager_node',
            name='role_manager_node',
            parameters=[{'robot_ids': robot_ids}],
            output='screen'
        )
    )

    # GUI:
    gui_script = os.path.expanduser('~/swarm/ros2_ws/src/gui_pkg/gui_pkg/gui.py')
    assert os.path.exists(gui_script), f"GUI script tidak ditemukan: {gui_script}"
    launch_nodes.append(
        ExecuteProcess(
            cmd=[sys.executable, gui_script],
            cwd=os.path.dirname(gui_script),
            output='screen',
            additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
        )
    )

    # Bridge Node
    bridge_node = Node(
        package='gui_pkg',  # Ganti dengan nama paket kamu jika diperlukan
        executable='bridge',  # Pastikan executable sudah benar
        name='bridge',
        output='screen',
    )
    launch_nodes.append(bridge_node)

    # RViz2
    launch_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    )

    return LaunchDescription(launch_nodes)
