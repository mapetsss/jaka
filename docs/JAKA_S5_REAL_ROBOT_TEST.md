# JAKA S5 实机验收清单

> 本流程会使真实机器人上电、使能和运动。必须由受过培训的人员在现场
> 执行，全程保持实体急停可用。软件停止和自动断电不能代替实体急停。

## 1. 环境与安全检查

- 停止 Gazebo、`jaka_driver` 和其他可能控制同一台机器人的节点。
- 确认机器人、底座、负载和末端工具安装正确。
- 在 JAKA App 中确认 TCP、负载和重心参数。
- 清空整个运动范围内的人员、线缆和障碍物。
- 验证实体急停，将全局速度限制设为 `5%` 或更低。
- 确认控制电脑与机器人同网段，且 IP 不冲突。

```bash
ping -c 4 <robot_ip>
```

## 2. 构建与静态检查

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to jaka_s5_controller
source install/setup.bash
colcon test --packages-select jaka_s5_controller
colcon test-result --verbose
```

## 3. 启动统一控制程序

```bash
ros2 launch jaka_s5_controller s5_joint_controller.launch.py \
  ip:=<robot_ip> \
  use_rviz:=true \
  diagnostics_enabled:=true \
  auto_power_off_on_exit:=true \
  auto_move_to_initial_on_start:=false
```

`moveit_server` 会登录、上电并使能机器人。若出现 SDK、通信、上电或使能
错误，立即停止测试。首次验收不得启用自动回初始位；确认命名位姿安全后，才可
使用 `auto_move_to_initial_on_start:=true` 单独验证启动自动回位。

## 4. 只读状态验证

暂不发送运动命令，先执行：

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /jaka_s5_controller/robot_diagnostics
ros2 action info /jaka_s5_controller/follow_joint_trajectory
```

确认六个关节名称、顺序和弧度值与 JAKA App 一致，诊断数据中无急停、碰撞、
软限位或控制器错误。

## 5. 低速运动验收

再次清场并确认急停人员就位后：

1. 将 Qt 界面速度和加速度设为 `5%` 或更低。
2. 先使用“同步当前姿态”，核对六轴反馈。
3. 将单次点动步长设为 `0.1 deg`，逐一验证 J1 至 J6 的正、负方向。
4. 每次点动后等待完成，确认其他五轴无指令性运动。
5. 将界面速度和加速度降至 `3%`，分别验证“初始位”和“预备位”按钮。
6. 验证六关节任务依次经过预备位和目标位，第一阶段失败后不得继续运动。
7. 验证运动期间重复提交被锁定，“停止运动”能中断点动和规划轨迹。

SDK 点动不经过 MoveIt 碰撞规划。每次点动前都必须单独确认该方向安全。

## 6. 退出与关机验收

1. 在无运动、轨迹运动和点动过程中分别验证关闭窗口。
2. 确认默认流程依次停止运动、退出伺服、下使能、调用 `power_off()` 并退出 SDK。
3. 在 JAKA App 中确认 `enabled=0` 且 `powered_on=0`。
4. 使用 `auto_power_off_on_exit:=false` 重复测试，确认不执行下使能和断电。

若关机服务报错或超时，应保持窗口打开，查看 SDK 错误并用 JAKA App
完成安全处理。

## 7. 验收记录

发布前记录以下内容：

- 测试日期、控制器版本、SDK 版本和 Git 提交号。
- 仿真、只规划、六轴点动、停止和关机的结果。
- 所有异常、终端日志与未解决限制。
