// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import static frc.robot.Constants.DriveConstants.*;

/**
 * CANDriveSubsystem - 维持为稳定的差速驱动子系统实现。
 *
 * 注意：之前文件中包含了半成品的 PathPlanner AutoBuilder 集成，
 * 这些代码会导致编译失败（缺少方法/字段）。我在这里移除那部分
 * 未完成的代码，保留驱动器初始化逻辑。后续我会给出一套可安全
 * 集成 PathPlanner 的步骤和示例代码，你可以按步骤合并到此子系统中。
 */
public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;
  // Kinematics and odometry
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  // Supplier which should provide the robot heading in degrees (e.g. from a Pigeon).
  // Default returns 0.0 until the caller wires a real supplier via setHeadingSupplier().
  private DoubleSupplier headingSupplier = () -> 0.0;
  // Suppliers to read encoder positions/velocities from the real hardware. These are
  // injected by RobotContainer (or tests) so this class does not need to depend on
  // a specific motor-controller API at compile time.
  // Units expected from suppliers: position -> motor rotations; velocity -> motor RPM
  private DoubleSupplier leftPositionSupplier = () -> 0.0;
  private DoubleSupplier leftVelocitySupplier = () -> 0.0;
  private DoubleSupplier rightPositionSupplier = () -> 0.0;
  private DoubleSupplier rightVelocitySupplier = () -> 0.0;

  public CANDriveSubsystem() {
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize odometry using current heading (if supplier is available).
    // DifferentialDriveOdometry constructors in this WPILib version expect
    // initial left/right wheel distances (we don't have encoders wired yet),
    // so pass zeros for distances for now.
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(headingSupplier.getAsDouble()), 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // Update odometry each loop. Currently wheel distances are not wired to encoders
    // yet, so we pass zero distances. After you hook up encoders, replace the
    // zeros with cumulative wheel distances in meters.
    Rotation2d heading = Rotation2d.fromDegrees(headingSupplier.getAsDouble());
    // TODO: replace with real distances from encoders
    // If encoder suppliers are injected, convert motor-rotations -> meters and update odometry
    double leftMeters = leftPositionSupplier.getAsDouble() * METERS_PER_MOTOR_REV;
    double rightMeters = rightPositionSupplier.getAsDouble() * METERS_PER_MOTOR_REV;
    odometry.update(heading, leftMeters, rightMeters);
  }

  /**
   * 简单的弧形驱动封装，供手动驾驶和简单自动命令使用。
   */
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Returns the current pose as tracked by odometry.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset the odometry to the specified pose.
   * This is useful to call at the start of autonomous to set the robot
   * pose to the starting pose of your path.
   */
  public void resetPose(Pose2d pose) {
    // Reset odometry to a known pose. The resetPosition API in this WPILib
    // version expects (Rotation2d, leftDistanceMeters, rightDistanceMeters, Pose2d).
    odometry.resetPosition(pose.getRotation(), 0.0, 0.0, pose);
  }

  /**
   * Returns current wheel speeds. TODO: wire encoders to return real values.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Convert encoder velocity (motor RPM) into meters/sec using METERS_PER_MOTOR_REV.
    // velocitySupplier is expected to return motor RPM (rotations per minute).
    double leftMps = (leftVelocitySupplier.getAsDouble() / 60.0) * METERS_PER_MOTOR_REV;
    double rightMps = (rightVelocitySupplier.getAsDouble() / 60.0) * METERS_PER_MOTOR_REV;
    return new DifferentialDriveWheelSpeeds(leftMps, rightMps);
  }

  /**
   * Set the drivetrain outputs in volts. This is the interface RamseteCommand
   * expects (via a voltage-output consumer). We convert volts to a -1..1 scale
   * roughly by dividing by 12V and call the motor set methods.
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // Many motor controllers accept a percent output; convert volts to percent-of-12V.
    leftLeader.set(leftVolts / 12.0);
    rightLeader.set(rightVolts / 12.0);
    drive.feed();
  }

  /**
   * Allow the robot code to inject a supplier that returns the robot heading in degrees.
   * Example usage (in RobotContainer):
   *   pigeon = new PigeonIMU(PIGEON_CAN_ID);
   *   driveSubsystem.setHeadingSupplier(() -> pigeon.getYaw());
   */
  public void setHeadingSupplier(DoubleSupplier supplier) {
    if (supplier != null) {
      headingSupplier = supplier;
    }
  }

  /**
   * Inject suppliers that return encoder readings from the hardware.
   * position suppliers should return motor rotations (cumulative).
   * velocity suppliers should return motor RPM.
   *
   * Example (RobotContainer with SparkMax API):
   *   driveSubsystem.setEncoderSuppliers(
   *     ()->leftLeader.getEncoder().getPosition(),
   *     ()->leftLeader.getEncoder().getVelocity(),
   *     ()->rightLeader.getEncoder().getPosition(),
   *     ()->rightLeader.getEncoder().getVelocity()
   *   );
   */
  public void setEncoderSuppliers(DoubleSupplier leftPos, DoubleSupplier leftVel, DoubleSupplier rightPos, DoubleSupplier rightVel) {
    if (leftPos != null) leftPositionSupplier = leftPos;
    if (leftVel != null) leftVelocitySupplier = leftVel;
    if (rightPos != null) rightPositionSupplier = rightPos;
    if (rightVel != null) rightVelocitySupplier = rightVel;
  }
}