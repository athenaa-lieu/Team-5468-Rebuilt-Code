// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  /** * 2026 Improvement: PoseEstimator instead of Odometry.
   * This allows for AprilTag vision updates to be "fused" with encoders.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(),
      // State standard deviations (Trust in: [x, y, theta])
      VecBuilder.fill(0.1, 0.1, 0.1), 
      // Vision standard deviations (Trust in: [x, y, theta])
      VecBuilder.fill(0.9, 0.9, 0.9));

  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update pose estimator with encoder and gyro data
    m_poseEstimator.update(getRotation2d(), getModulePositions());
  }

  /**
   * 2026 Field-Relative Drive Logic
   * @param fieldRelative If true, +X is always away from the BLUE Alliance Wall.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Apply max speeds from constants
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // 2026 Alliance Correction: 
    // WPILib coordinates are always Blue-Relative. If we are Red, we flip inputs.
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && fieldRelative) {
        xSpeedDelivered *= -1;
        ySpeedDelivered *= -1;
    }

    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Helper to fetch all module positions for PoseEstimator/Odometry */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /** Returns Rotation2d with correct NavX inversion logic */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(DriveConstants.kGyroReversed ? m_gyro.getAngle() : -m_gyro.getAngle());
  }

  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * 2026 Vision Integration: Call this from your Vision Subsystem 
   * when an AprilTag is detected to correct the robot's field position.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }
}