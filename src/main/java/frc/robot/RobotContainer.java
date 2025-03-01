// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.ElevatorConstants.CoralLevels;
import frc.robot.constants.PoseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.AprilTagRecognition;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedLights;
import frc.robot.vision.AprilTagLocalization;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric driveRobCentric =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private AprilTagLocalization m_aprilTagLocalization =
      new AprilTagLocalization(
          m_drivetrain::getPose2d,
          m_drivetrain::resetPose,
          m_drivetrain::addVisionMeasurement,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_BACKL,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_ELEVATE,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_FRONTR
          );

  private final LedLights m_ledLights = LedLights.getInstance();
  private final Climbing m_climbing = new Climbing();
  private final AlgaeRollers m_algaeRollers = new AlgaeRollers();
  private final CoralRollers m_coralRollers = new CoralRollers();
  private final CoralPivot m_coralPivot = new CoralPivot();
  private final AlgaePivot m_algaePivot = new AlgaePivot();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  private final Elevator m_elevator = new Elevator();
  private final CommandFactory m_commandFactory =
      new CommandFactory(
          m_drivetrain,
          m_algaeRollers,
          m_climbing,
          m_elevator,
          m_coralRollers,
          m_ledLights,
          m_coralPivot,
          m_algaePivot);
  private final AprilTagRecognition m_aprilTagRecognition =
      new AprilTagRecognition(m_commandFactory);

  public RobotContainer() {

    NamedCommands.registerCommand(
        "Raise Elevator to position 1", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L1));
    NamedCommands.registerCommand(
        "Raise Elevator to position 2", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L2));
    NamedCommands.registerCommand(
        "Raise Elevator to position 3", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L3));
    NamedCommands.registerCommand(
        "Raise Elevator to position 4", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L4));
    NamedCommands.registerCommand(
        "Raise Elevator to position Coral Station", m_commandFactory.elevatorPivotAndIntake());

    configureBindings();
    configureCoDriverControls();

    // Adds a auto chooser to Shuffle Board to choose autos
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private boolean yIsNotPressed() {
    SmartDashboard.putBoolean("Y Pressed", yIsPressed());
    return !yIsPressed();
  }

  private boolean yIsPressed() {
    return m_driverController.y().getAsBoolean();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    m_drivetrain.setDefaultCommand(
        m_drivetrain.gasPedalCommand(
            m_driverController::getRightTriggerAxis,
            m_driverController::getRightX,
            m_driverController::getLeftY,
            m_driverController::getLeftX));

    m_driverController.x().whileTrue(m_aprilTagRecognition.getAprilTagCommand());
    m_driverController.y().whileTrue(m_commandFactory.algaePivotAndOutake());

    // m_driverController.povUp().and(this::yIsNotPressed).onTrue(m_elevator.raiseToNextPosition());
    // m_driverController.povDown().and(this::yIsNotPressed).onTrue(m_elevator.lowerToNextPosition());

    // Elevator commands
    // m_driverController.povUp().and(this::yIsPressed).whileTrue(m_elevator.trimUp());
    // m_driverController.povUp().and(this::yIsPressed).onFalse(m_elevator.stopMotors());

    // m_driverController.povDown().and(this::yIsPressed).whileTrue(m_elevator.trimDown());
    // m_driverController.povDown().and(this::yIsPressed).onFalse(m_elevator.stopMotors());

    /*m_driverController
        .povUp()
        .and(this::yIsNotPressed)
        .onTrue(m_elevator.goToCoralHeightPosition(CoralLevels.L1));

    m_driverController.povRight().onTrue(m_elevator.goToCoralHeightPosition(CoralLevels.L2));

    m_driverController
        .povDown()
        .and(m_driverController.y().negate())
        .onTrue(m_elevator.goToCoralHeightPosition(CoralLevels.L3));

    m_driverController.povLeft().onTrue(m_elevator.goToCoralHeightPosition(CoralLevels.L4));*/

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // m_driverController.x().onTrue(m_coralPivot.goToLowerSetpoint());
    // m_driverController.y().onTrue(m_coralPivot.goToUpperSetpoint());

    // m_driverController.y().onTrue(m_algaeRollers.feedIn());
    // m_driverController.y().onFalse(m_algaeRollers.stop());

    // m_driverController.b().onTrue(m_algaeRollers.feedOut());
    // m_driverController.b().onFalse(m_algaeRollers.stop());

    // this is buppers for coral station
    m_driverController
        .rightBumper()
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.rightCoralStationPosition2::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake()));
    m_driverController
        .leftBumper()
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.leftCoralStationPosition2::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake()));
  }

  private void configureCoDriverControls() {
    // Setup codriver's controlls
    m_coDriverController
        .a()
        .whileTrue(m_coralRollers.rollInCommand())
        .onFalse(m_coralRollers.stopCommand());
    m_coDriverController
        .b()
        .whileTrue(m_coralRollers.rollOutCommand())
        .onFalse(m_coralRollers.stopCommand());
    m_coralRollers
        .getCoralTrigger()
        .onTrue(rumbleCommand(m_coDriverController, RumbleType.kBothRumble, 1.0, Seconds.of(0.5)));
  }

  private Command rumbleCommand(
      CommandXboxController xboxController,
      RumbleType rumbleType,
      double rumbleStrength,
      Time rumbleTime) {
    Command wait = Commands.waitTime(rumbleTime);
    Command stopRumble =
        Commands.runOnce(
            () -> {
              xboxController.setRumble(rumbleType, 0.0);
            });

    return Commands.runOnce(
            () -> {
              xboxController.setRumble(rumbleType, rumbleStrength);
            })
        .andThen(wait)
        .andThen(stopRumble);
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
