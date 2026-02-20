// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;


public class LaunchSequence extends SequentialCommandGroup {
  /** 创建新的发射序列 */
  
  /**
   * 构造函数 - 创建发射序列命令组
   * 
   * @param fuelSubsystem 燃料子系统实例
   * 
   * 构造函数主要职责：
   * 1. 定义发射序列的两个阶段及其执行顺序
   * 2. 为第一阶段（预热）设置时间限制
   * 3. 确保发射逻辑的安全性和可预测性
   */
  public LaunchSequence(CANFuelSubsystem fuelSubsystem) {
    // 在addCommands()调用中添加您的命令，例如：

    // 添加命令序列：
    addCommands(
        // 阶段1：预热发射器（旋转加速）
        // SpinUp命令启动发射滚轮，使其达到发射所需的速度
        // withTimeout(FuelConstants.SPIN_UP_SECONDS)将预热阶段限制为固定时间
        // 这是安全关键：确保发射滚轮达到足够速度后再送入燃料
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        
        // 阶段2：发射燃料
        // Launch命令启动给料滚轮，将燃料送入高速旋转的发射滚轮
        // 此命令没有时间限制，将持续执行直到被主动中断
        // 在RobotContainer中，这个序列被绑定到whileTrue触发器，因此当按钮松开时会被中断
        new Launch(fuelSubsystem)
    );
    

  }


}