import launch
import launch_ros
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明参数，是否创建新的海龟
    declare_spawn_turtle = launch.actions.DeclareLaunchArgument(
        'spawn_turtle', default_value='False', description = '是否生成新的海龟')
    spawn_turtle = launch.substitutions.LaunchConfiguration("spawn_turtle")


    action_turtlesim = launch_ros.actions.Node(
        package="turtlesim", executable="turtlesim_node", output="screen"
    )
    # 给日志输出和服务调用添加条件
    action_executeprocess = launch.actions.ExecuteProcess(
        condition=IfCondition(spawn_turtle),
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/srv/Spawn', '{x:1, y:1}']
    )

    action_log_info = launch.actions.LogInfo(condition=IfCondition(spawn_turtle), msg = "使用executeprocess 来调用服务生成海龟")
    # 利用计时器动作实现依次启动
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
        launch.actions.TimerAction(period=3.0, actions=[action_executeprocess]),
    ])

    # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        declare_spawn_turtle,
        action_turtlesim,
        action_group
    ])

    return launch_description
