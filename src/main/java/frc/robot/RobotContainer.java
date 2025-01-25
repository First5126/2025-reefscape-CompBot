// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AprilTagLocalization;
import frc.robot.vision.AprilTagLocalizationConstants;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform*/
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric driveRobCentric = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_codriver_controller = new CommandXboxController(1);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser(); 
  private AprilTagLocalization m_aprilTagLocalization = new AprilTagLocalization(
    m_drivetrain::getPose2d,
    m_drivetrain::resetPose,
    m_drivetrain::addVisionMeasurement,
    AprilTagLocalizationConstants.LIMELIGHT_DETAILS
  );

  public RobotContainer() {
      configureBindings();
      configureCoDriverControls();

      // Adds a auto chooser to Shuffle Board to choose autos
      SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    
    m_drivetrain.setDefaultCommand(m_drivetrain.gasPedalCommand(
        m_driverController::getRightTriggerAxis,
        m_driverController::getRightX,
        m_driverController::getLeftY,
        m_driverController::getLeftX
    ));

    m_driverController.rightBumper().onTrue(m_drivetrain.zero_pidgeon());
    m_driverController.y().whileTrue(m_aprilTagLocalization.setTrust(true));
    m_driverController.y().onFalse(m_aprilTagLocalization.setTrust(false));
  }

  private void configureCoDriverControls() {
    // Setup codriver's controlls
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}