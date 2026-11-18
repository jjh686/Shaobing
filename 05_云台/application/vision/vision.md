# Vision 应用模块

auto_aim_boot.c/.h：从application/cmd原样迁移的独立视觉自主启动状态机。
入口：AutoAimBootInit、AutoAimBootUpdate；原有状态枚举和调试变量名称保持不变。

遥控/视觉切换完整保留在application/cmd/robot_cmd.c，文件内容与整理前一致。
USB/SP收发、会话管理及姿态回传仍由application/communication负责。
电机驱动、机械限位、闭环控制及现有配置保持原位。
本次仅整理目录和构建路径，不改变初始化顺序、调用周期和运行逻辑。
