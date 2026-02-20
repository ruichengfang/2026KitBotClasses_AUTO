// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;


public class AutoDrive extends Command {
  // 驱动子系统引用，用于控制机器人移动
  private final CANDriveSubsystem driveSubsystem;
  
  // 驱动参数：前进/后退速度
  private final double xSpeed;
  
  // 驱动参数：旋转速度
  private final double zRotation;

  /**
   * 构造函数 - 创建自动驱动命令
   * 
   * @param driveSystem 驱动子系统实例，不能为null
   * @param xSpeed 前进/后退速度，范围[-1.0, 1.0]
   *               正值前进，负值后退
   * @param zRotation 旋转速度，范围[-1.0, 1.0]
   *                  正值顺时针旋转，负值逆时针旋转
   * 
   * 构造函数主要职责：
   * 1. 声明子系统依赖关系
   * 2. 保存子系统引用和驱动参数
   */
  public AutoDrive(CANDriveSubsystem driveSystem, double xSpeed, double zRotation) {
    // 使用addRequirements()声明命令所需的子系统
    // 防止命令执行期间其他命令同时使用该子系统
    addRequirements(driveSystem);
    
    // 保存子系统引用
    driveSubsystem = driveSystem;
    
    // 保存驱动参数
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    // 调用驱动子系统的弧线驱动方法
    // 将构造函数中设置的速度参数传递给机器人
    driveSubsystem.driveArcade(xSpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    // 停止所有驱动电机
    driveSubsystem.driveArcade(0, 0);

  }


  @Override
  public boolean isFinished() {
    // 返回false表示命令永远不会自动结束
    // 命令将一直执行直到：
    // 1. 被调度器取消
    // 2. 被更高优先级的命令中断
    // 3. 机器人模式改变（如从自动模式切换到遥控模式）
    return false;
  }

}