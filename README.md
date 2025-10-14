# FishRos
跟着小鱼（桑欣）的书籍《ROS2机器人开发从入门到实践》敲一下各章的代码。
如果你有查看执行效果的需要，请克隆本仓库后，删除各_ws文件夹中的install, build, log文件夹，然后重新colcon build构建项目，并source install/setup.bash后，再执行你想执行的指令。


### 6.3 添加物理属性
运行命令如下:

```bash
ros2 launch fishbot_description display_robot.launch.py model:=src/fishbot_description/urdf/fishbot/fishbot.urdf.xacro
```
