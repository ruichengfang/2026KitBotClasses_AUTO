// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;


public class Drive extends Command {
  /** 创建新的驾驶命令 */
  
  // 驱动子系统引用
  CANDriveSubsystem driveSubsystem;
  
  // Xbox控制器引用，用于获取驾驶员输入
  CommandXboxController controller;

  /**
   * 构造函数 - 创建驾驶命令实例
   * 
   * @param driveSystem 驱动子系统实例
   * @param driverController Xbox控制器实例
   * 
   * 构造函数主要职责：
   * 1. 声明子系统依赖关系，确保命令独占访问
   * 2. 保存子系统和控制器引用供后续使用
   * 3. 建立控制器到驱动系统的映射关系
   */
  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    // 使用addRequirements()声明子系统依赖关系
    // 这将防止多个命令同时访问驱动子系统
    addRequirements(driveSystem);
    
    // 保存子系统引用
    driveSubsystem = driveSystem;
    
    // 保存控制器引用
    controller = driverController;
  }



  @Override
  public void initialize() {

  }

  /**
   * 执行方法 - 命令调度期间每个调度周期调用
   * 
   * 功能：读取控制器输入并驱动机器人
   * 核心逻辑：
   * 1. 获取控制器的左Y轴值（前进/后退）
   * 2. 获取控制器的右X轴值（旋转）
   * 3. 应用反转和缩放因子
   * 4. 将处理后的值传递给驱动子系统
   * 
   * 关键设计决策：
   * - 控制器Y轴值取反：摇杆向前推（负值）时机器人向前（正值）
   * - 控制器X轴值取反：摇杆向右推（正值）时机器人顺时针旋转（负值）
   * - 应用缩放因子：使机器人运动更平滑可控
   * - 缩放因子来自Constants.OperatorConstants
   */
  @Override
  public void execute() {
    // 驱动命令的核心执行逻辑
    
    // 获取左摇杆Y轴值（前进/后退控制）
    // -1.0：摇杆完全向前（推离驾驶员）
    // 0.0：摇杆居中
    // 1.0：摇杆完全向后（拉向驾驶员）
    double leftY = -controller.getLeftY();  // 取反使控制更符合直觉
    
    // 获取右摇杆X轴值（旋转控制）
    // -1.0：摇杆完全向左
    // 0.0：摇杆居中
    // 1.0：摇杆完全向右
    double rightX = -controller.getRightX(); // 取反使控制更符合直觉
    
    // 应用缩放因子
    // DRIVE_SCALING：前进/后退缩放，通常小于1.0以降低灵敏度
    // ROTATION_SCALING：旋转缩放，通常小于1.0以使旋转更平缓
    double scaledLeftY = leftY * DRIVE_SCALING;
    double scaledRightX = rightX * ROTATION_SCALING;
    
    // 将处理后的控制输入传递给驱动子系统
    driveSubsystem.driveArcade(scaledLeftY, scaledRightX);
  }

  /**
   * 结束方法 - 命令结束时调用（正常结束或被中断）
   * 
   * @param interrupted 指示命令是如何结束的
   *                   true：被其他命令中断
   *                   false：正常结束
   * 
   * 功能：执行清理操作，停止机器人
   * 安全关键：无论命令如何结束，都应确保机器人停止
   * 防止命令结束后机器人继续移动造成危险
   */
  @Override
  public void end(boolean interrupted) {
    // 停止机器人驱动（设置速度和旋转都为0）
    driveSubsystem.driveArcade(0, 0);
    
    // 可选：根据中断状态执行不同的清理操作
    // if (interrupted) {
    //   System.out.println("Drive command was interrupted");
    //   // 可能记录中断原因或执行紧急停止序列
    // } else {
    //   System.out.println("Drive command ended normally");
    // }
  }

  /**
   * 完成判断方法 - 决定命令何时结束
   * 
   * @return 如果命令应该结束返回true，否则返回false
   * 
   * 设计选择：
   * - 返回false表示命令永远不会自动结束
   * - 命令将一直执行直到被主动取消
   * - 这适合作为默认命令：只要机器人处于遥控模式，就持续响应控制器输入
   */
  @Override
  public boolean isFinished() {
    // 返回false，命令持续执行直到被取消
    // 这种设计适用于：
    // 1. 默认驾驶命令：在机器人启用期间持续运行
    // 2. 手动控制场景：需要持续响应控制器输入
    
    // 注意：通常驾驶命令应该是常驻命令，由调度器或模式切换来控制其生命周期
    
    return false;
  }


}