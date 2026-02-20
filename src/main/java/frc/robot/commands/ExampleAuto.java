package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

// 注意：考虑在线使用此命令，而不是编写子类。有关更多信息，请参阅：
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * ExampleAuto类 - 示例自动程序命令组
 * 
 * 功能：这是一个顺序命令组（SequentialCommandGroup），按顺序执行一系列命令
 * 实现一个简单的自动程序，包含两个步骤：
 * 1. 向后行驶0.25秒
 * 2. 启动发射器并持续发射燃料10秒
 * 
 * 设计模式：使用命令组合模式，将多个简单命令组合成复杂的自动程序
 */
public class ExampleAuto extends SequentialCommandGroup {
  
  /**
   * 构造函数 - 创建示例自动程序
   * 
   * @param driveSubsystem 驱动子系统实例，用于控制机器人移动
   * @param ballSubsystem 燃料子系统实例，用于控制发射系统
   *                       注意：参数名称为ballSubsystem，但实际类型是CANFuelSubsystem
   *                       可能是命名不一致，建议重命名为fuelSubsystem更清晰
   * 
   * 构造函数主要职责：
   * 1. 定义自动程序的执行序列
   * 2. 使用addCommands()方法添加并按顺序执行命令
   * 3. 使用命令装饰器（如withTimeout）控制命令执行时间
   */
  public ExampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    // 在addCommands()调用中添加您的命令，例如：
    // addCommands(new FooCommand(), new BarCommand());
    
    // 添加命令序列：
    addCommands(
      // 命令1：向后行驶0.25秒
      // AutoDrive命令以0.5的速度前进（正数），0的旋转速度
      // 注意：注释说"向后"，但速度参数为0.5（正数），需要检查电机方向配置
      // 如果0.5确实是向前，那么注释可能是错误的，或者电机配置为反转
      // withTimeout(.25)将命令限制为执行0.25秒，然后自动结束
      new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(.25),
      
      // 命令2：启动发射器并发射燃料10秒
      // Launch命令启动发射器系统
      // withTimeout(10)将命令限制为执行10秒
      // 注释说明：前1秒旋转加速发射器，然后发射燃料9秒，总计10秒
      new Launch(ballSubsystem).withTimeout(10)
    );
    
    /**
     * 自动程序流程详解：
     * 
     * 第1阶段：移动阶段（0.25秒）
     * - 机器人以50%的前进速度（xSpeed=0.5）直线行驶
     * - 不旋转（zRotation=0.0）
     * - 持续0.25秒后自动停止
     * 
     * 第2阶段：发射阶段（10秒）
     * - 启动燃料发射系统
     * - 假设Launch命令内部包含：
     *   1. 前1秒：预热/旋转加速发射滚轮（达到发射速度）
     *   2. 后9秒：启动给料滚轮，开始发射燃料
     * - 持续10秒后自动停止
     * 
     * 总自动程序时间：0.25 + 10 = 10.25秒
     */
  }
}