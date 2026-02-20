// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;


public class SpinUp extends Command {

  // 燃料子系统引用
  CANFuelSubsystem fuelSubsystem;

  /**
   * 构造函数 - 创建预热命令实例
   * 
   * @param fuelSystem 燃料子系统实例，不能为null
   * 
   * 构造函数主要职责：
   * 1. 声明子系统依赖关系，确保命令独占访问
   * 2. 保存子系统引用供后续使用
   */
  public SpinUp(CANFuelSubsystem fuelSystem) {
    // 声明命令所需的子系统，防止多个命令同时访问同一子系统
    addRequirements(fuelSystem);
    // 保存子系统引用
    this.fuelSubsystem = fuelSystem;
  }


  @Override
  public void initialize() {

    fuelSubsystem.setIntakeLauncherRoller(
        SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    

    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Launching spin-up feeder value", SPIN_UP_FEEDER_VOLTAGE));
  }

  @Override
  public void execute() {

  }

  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {

    return false;
  }


}