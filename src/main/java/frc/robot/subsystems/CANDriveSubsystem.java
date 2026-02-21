// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * CANDriveSubsystem类 - 机器人驱动子系统
 *
 * 功能：管理机器人的驱动系统，使用CAN总线连接的REV Spark Max控制器控制有刷电机 实现差速驱动，支持弧形驱动控制模式
 */
public class CANDriveSubsystem extends SubsystemBase {
    // 左侧电机控制器：一个主控制器和一个跟随控制器

    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    // 右侧电机控制器：一个主控制器和一个跟随控制器
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;

    // WPILib提供的差速驱动类，处理左右两侧电机的协同控制
    private final DifferentialDrive drive;

    // 差速驱动运动学对象，用于速度转换
    private static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(0.6); // 轮间距（米）

    // 差速驱动里程计，用于跟踪机器人的位置和姿态
    private final DifferentialDriveOdometry odometry;

    /**
     * 构造函数 - 初始化驱动子系统
     *
     * 主要功能： 1. 创建并配置所有Spark Max电机控制器 2. 设置电压补偿和电流限制 3. 配置电机跟随模式 4.
     * 设置电机反转以保证两侧方向一致
     */
    public CANDriveSubsystem() {
        RobotConfig autoconfig = null;
        try {
            autoconfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                autoconfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        // 创建有刷电机控制器实例
        // 左侧主电机（控制器ID来自Constants.DriveConstants）
        leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
        // 左侧跟随电机
        leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
        // 右侧主电机
        rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
        // 右侧跟随电机
        rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

        // 创建差速驱动实例，传入左右两侧的主电机控制器
        // 跟随电机会自动跟随对应的主电机
        drive = new DifferentialDrive(leftLeader, rightLeader);

        // 设置CAN通信超时时间为250毫秒
        // 由于本程序仅在构造函数中配置参数，可以使用较长的超时时间
        // 如果在运行期间频繁配置参数，则需要更短的超时时间以避免阻塞操作
        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);

        // 创建Spark Max配置对象，用于统一配置所有电机控制器
        SparkMaxConfig config = new SparkMaxConfig();
        // 设置电压补偿为12V，确保在不同电池电压下性能一致
        // 代价是满电时最高速度略有降低
        config.voltageCompensation(12);
        // 设置智能电流限制，防止断路器跳闸
        // 限制值来自Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT
        config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

        // leftFollower
        config.follow(leftLeader);
        // 应用配置到左侧跟随电机，重置安全参数并持久化配置
        // ResetMode.kResetSafeParameters：重置安全相关参数
        // PersistMode.kPersistParameters：将配置持久化到控制器闪存
        leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // rightFollower
        config.follow(rightLeader);
        // 应用配置到右侧跟随电机
        rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 禁用跟随模式，准备配置主电机
        config.disableFollowerMode();

        // rightLeader
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // leftLeader
        config.inverted(true);
        // 应用配置到左侧主电机
        leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 初始化里程计，初始位置为原点，初始旋转角度为0
        odometry = new DifferentialDriveOdometry(
                new Rotation2d(),
                0.0, // 左侧轮初始位置
                0.0, // 右侧轮初始位置
                new Pose2d() // 初始位置
        );
    }

    @Override
    public void periodic() {
        // 更新里程计，使用当前的旋转角度和轮速
        // 注意：这里假设使用了陀螺仪和编码器，实际应用中需要根据硬件配置调整
        odometry.update(
                new Rotation2d(), // 这里应替换为实际的陀螺仪角度
                leftLeader.getEncoder().getPosition(), // 左侧轮位置
                rightLeader.getEncoder().getPosition() // 右侧轮位置
        );
    }

    /**
     * 获取当前机器人的姿势（位置和方向）
     *
     * @return 当前机器人的Pose2d对象
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * 将机器人的里程计重置为给定的姿势
     *
     * @param pose 要重置到的Pose2d对象
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(
                pose.getRotation(),
                leftLeader.getEncoder().getPosition(),
                rightLeader.getEncoder().getPosition(),
                pose
        );
    }

    /**
     * 获取当前机器人的相对速度（机器人坐标系下）
     *
     * @return 当前机器人的ChassisSpeeds对象
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        // 获取左右轮速度
        double leftSpeed = leftLeader.getEncoder().getVelocity();
        double rightSpeed = rightLeader.getEncoder().getVelocity();

        // 使用差速驱动运动学计算底盘速度
        // 注意：这里假设DRIVE_KINEMATICS已在Constants.DriveConstants中定义
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
        return DRIVE_KINEMATICS.toChassisSpeeds(wheelSpeeds);
    }

    /**
     * 以机器人相对速度驱动机器人
     *
     * @param speeds 机器人相对的ChassisSpeeds对象
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // 将底盘速度转换为轮速
        DifferentialDriveWheelSpeeds wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(speeds);

        // 设置电机速度
        leftLeader.set(wheelSpeeds.leftMetersPerSecond);
        rightLeader.set(wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * 控制机器人移动的command
     *
     * @param xSpeed 前进/后退速度，范围[-1.0, 1.0] 正值前进，负值后退
     * @param zRotation 旋转速度，范围[-1.0, 1.0] 正值顺时针旋转，负值逆时针旋转
     *
     * 功能：将控制输入转换为电机速度，实现机器人的移动控制 弧形驱动模式：前进/后退和旋转同时控制，适合单摇杆操作
     */
    public void driveArcade(double xSpeed, double zRotation) {
        // 调用WPILib DifferentialDrive的arcadeDrive方法
        // 该方法会自动处理电机混合计算，将速度指令发送到电机控制器
        drive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * 可选的附加方法（如果需要可以添加）：
     *
     * 1. tankDrive(double leftSpeed, double rightSpeed) - 坦克式驱动 2.
     * curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) -
     * 曲率驱动 3. stop() - 停止所有电机 4. setMaxOutput(double maxOutput) - 设置最大输出限制 5.
     * getLeftEncoderPosition() - 获取左侧编码器位置 6. getRightEncoderPosition() -
     * 获取右侧编码器位置 7. resetEncoders() - 重置编码器
     */
}
