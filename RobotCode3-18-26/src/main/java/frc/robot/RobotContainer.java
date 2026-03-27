// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.ShootDelay;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeMove;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeMove m_intakemove = new IntakeMove();

  // Controllers
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_articulatorController = new XboxController(OIConstants.kArticulatorControllerPort);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default Command: Field-Relative Driving
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                // Forward/Backward: Xbox Y is negative when pushed forward
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                // Left/Right: Xbox X is negative when pushed left
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                // Rotation
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true), // Field-Relative set to TRUE for 2026
            m_robotDrive));
  }

  private void configureButtonBindings() {
    /* --- DRIVER CONTROLLER --- */

    // X-Pattern (Brake Mode)
    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    //     .whileTrue(new RunCommand(m_robotDrive::setX, m_robotDrive));

    // Zero Heading (Reset Field Orientation)
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // Robot-Relative Toggle (Hold Right Bumper for "Standard" driving)
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false), 
            m_robotDrive));

    /* --- ARTICULATOR CONTROLLER --- */

    // Intake Rollers
    new JoystickButton(m_articulatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new StartEndCommand(m_intake::intake, m_intake::stop, m_intake));

    // Indexer Feed
    new Trigger(() -> m_articulatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(new StartEndCommand(m_indexer::feed, m_indexer::stop, m_indexer));

    // Indexer Reverse
    new JoystickButton(m_articulatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new StartEndCommand(m_indexer::reverse, m_indexer::stop, m_indexer));

    // Shooter (Run at RPM)
    new Trigger(() -> m_articulatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(new StartEndCommand(
            () -> m_shooter.runAtRPM(Constants.SHOOTER_RPM),
            m_shooter::stop,
            m_shooter));

    // Intake Pivot In + Rollers
    new JoystickButton(m_articulatorController, XboxController.Button.kA.value)
         .whileTrue(new RunIntakeIn(m_intakemove).alongWith(new RunIntake(m_intake)));

    // Intake Pivot Out
    new JoystickButton(m_articulatorController, XboxController.Button.kY.value)
        .whileTrue(new RunIntakeOut(m_intakemove));

    // Shoot Sequence
    new JoystickButton(m_articulatorController, XboxController.Button.kX.value)
        .whileTrue(ShootDelay.create(m_shooter, m_indexer));
  }

  public Command getAutonomousCommand() {
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose,
    //     DriveConstants.kDriveKinematics,
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // Reset pose to start of path
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        return null; // TODO: Return your autonomous command here.
  }
}