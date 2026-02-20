// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;


public class RobotContainer {
  // 机器人子系统 - 机器人功能的核心模块
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

  // 驾驶员控制器 - 用于控制机器人移动
  // 端口号来自Constants.OperatorConstants.DRIVER_CONTROLLER_PORT
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // 操作员控制器 - 用于控制燃料系统和其他辅助功能
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // 自动程序选择器 - 用于在Dashboard上选择不同的自动程序
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * 机器人容器构造函数
   * 
   * 功能：初始化机器人容器，执行以下操作：
   * 1. 配置按钮和命令的绑定关系
   * 2. 设置自动程序选择器的选项
   * 
   * 这是机器人启动时调用的第一个方法之一，完成所有初始化工作
   */
  public RobotContainer() {
    // 配置按钮与命令的绑定关系
    configureBindings();

    // 设置Dashboard上显示的自动模式选项
    // 如果添加额外的自动模式，可以使用autoChooser.addOption添加更多行
    autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
  }


  private void configureBindings() {

    // 操作员左肩键（LB）按下时：收集球
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    
    // 操作员右肩键（RB）按下时：发射球
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    
    // 操作员A按钮按下时：吐球
    // 反向排出，用于清除卡住的球
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    /**
     * 子系统默认命令配置：
     * 
     * 默认命令在没有其他命令使用该子系统时自动运行
     * 这提供了系统的"空闲状态"行为
     */


    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    // 设置燃料子系统的默认命令为停止命令
    // 这确保没有按钮按下时，燃料系统自动停止
    // 使用run()工厂方法创建简单命令，调用fuelSubsystem.stop()
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  /**
   * 获取自动命令方法
   * 
   * 功能：将选择的自动命令传递给主Robot类
   * 在自动模式期间，调度器将运行此命令
   * 
   * @return 在自动模式下运行的命令
   * 
   * 调用时机：机器人在自动模式开始时调用此方法获取要执行的命令
   * 选择器允许驾驶员在比赛前通过Dashboard选择不同的自动策略
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }


}