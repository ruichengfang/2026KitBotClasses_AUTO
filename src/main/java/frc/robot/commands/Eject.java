// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;


public class Eject extends Command {


  CANFuelSubsystem fuelSubsystem;


  public Eject(CANFuelSubsystem fuelSystem) {
    // 声明命令所需的子系统，防止多个命令同时访问同一子系统
    addRequirements(fuelSystem);
    // 保存子系统引用
    this.fuelSubsystem = fuelSystem;
  }

  /**
   * 初始化方法 - 命令首次被调度时调用一次
   * 
   * 功能：设置滚轮以排出模式运行
   * 关键设计：
   * 1. 从SmartDashboard获取调优参数，如果没有设置则使用默认值
   * 2. 对电压值取负号（乘以-1），使滚轮反向旋转
   * 3. 设置两个滚轮的电压，启动排出过程
   * 
   * 电压获取逻辑：
   * - SmartDashboard.getNumber(key, defaultValue)
   *   - 首先尝试从SmartDashboard读取指定键的值
   *   - 如果不存在，则使用默认值（来自Constants.FuelConstants）
   * - 这种设计允许操作员实时调整参数而无需重新部署代码
   */
  @Override
  public void initialize() {
    // 负号表示反向旋转，吐球
    fuelSubsystem.setIntakeLauncherRoller(
        -1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));//应该是可以从smartdashboard手动调节，然后读取，便于场上实时调节
    
    // 同样使用负号实现反向旋转
    fuelSubsystem.setFeederRoller(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));

  }


  @Override
  public void execute() {

  }

  /**
   * 结束方法 - 命令结束时调用（正常结束或被中断）
   * 
   * @param interrupted 指示命令是如何结束的
   *                   true：被其他命令中断
   *                   false：正常结束（虽然这个命令永远不会正常结束）
   * 
   * 功能：执行清理操作，停止所有滚轮
   * 安全关键：无论命令如何结束，都应确保滚轮停止
   * 防止命令结束后滚轮继续旋转造成危险
   */
  @Override
  public void end(boolean interrupted) {
    // 停止发射
    fuelSubsystem.setIntakeLauncherRoller(0);

    fuelSubsystem.setFeederRoller(0);
    
  }

  @Override
  public boolean isFinished() {

    return false;
  }


}