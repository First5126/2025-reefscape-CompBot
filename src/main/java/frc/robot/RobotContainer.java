// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.CoralLevels;
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
import frc.robot.subsystems.RecordInputs;
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
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_FRONTR);

  private final LedLights m_ledLights = LedLights.getInstance();
  private final Climbing m_climbing = new Climbing();
  private final AlgaeRollers m_algaeRollers = new AlgaeRollers();
  private final CoralRollers m_coralRollers = new CoralRollers();
  private final CoralPivot m_coralPivot = new CoralPivot();
  private final AlgaePivot m_algaePivot = new AlgaePivot();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  private final Elevator m_elevator = new Elevator();
  private final RecordInputs m_recordInputs = new RecordInputs();
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

    /*NamedCommands.registerCommand(
        "Raise Elevator to position 1", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L1));
    NamedCommands.registerCommand(
        "Raise Elevator to position 2", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L2));
    NamedCommands.registerCommand(
        "Raise Elevator to position 3", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L3));
    NamedCommands.registerCommand(
        "Raise Elevator to position 4", m_commandFactory.elevatorPivotAndOutTake(CoralLevels.L4));
    NamedCommands.registerCommand(
        "Raise Elevator to position Coral Station", m_commandFactory.elevatorPivotAndIntake());*/

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

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.x().whileTrue(m_aprilTagRecognition.getAprilTagCommand());

    m_driverController.b().onTrue(m_aprilTagLocalization.setTrust(true));
    m_driverController.b().onFalse(m_aprilTagLocalization.setTrust(false));

    m_driverController.a().onTrue(m_drivetrain.zero_pidgeon());
    // Bumpers to coral station

    // d-pad for side selection
    m_driverController.povLeft().onTrue(m_recordInputs.setLeftSideCoralStation());
    m_driverController.povRight().onTrue(m_recordInputs.setRightSideCoralStation());

    // m_driverController.a().onTrue(m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION));

    // right bumper left goto
    /*
    m_driverController
        .rightBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.rightCoralStationPosition3::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // right bumper left return
    m_driverController
        .rightBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose, // TODO: add a return position constant
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));

    // right bumper right goto
    m_driverController
        .rightBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.rightCoralStationPosition1::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // right bumper right return
    m_driverController
        .rightBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));

    // left bumper left side goto
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.leftCoralStationPosition3::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // left bumper left side return
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));

    // left bumper right side goto
    m_driverController
        .leftBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.leftCoralStationPosition1::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // left bumper right side return
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.getCoralTrigger())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));*/
  }

  private void configureCoDriverControls() {
    // Setup codriver's controlls

    // debug
    // m_coDriverController.a().onTrue(m_coralPivot.)

    // Elevator commands
    m_coDriverController.povUp().and(m_coDriverController.y()).whileTrue(m_elevator.trimUp());
    m_coDriverController.povUp().and(m_coDriverController.y()).onFalse(m_elevator.stopMotors());

    m_coDriverController.povDown().and(m_coDriverController.y()).whileTrue(m_elevator.trimDown());
    m_coDriverController.povDown().and(m_coDriverController.y()).onFalse(m_elevator.stopMotors());

    m_coDriverController
        .povUp()
        .and(this::yIsNotPressed)
        .onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L1));

    m_coDriverController.povRight().onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L2));

    m_coDriverController
        .povDown()
        .and(m_coDriverController.y().negate())
        .onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L3));

    m_coDriverController.povLeft().onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L4));

    // intakes/outtakes

    m_coDriverController.leftBumper().onTrue(m_algaeRollers.feedIn());
    m_coDriverController.leftBumper().onFalse(m_algaeRollers.stop());

    m_coDriverController.rightBumper().onTrue(m_algaeRollers.feedOut());
    m_coDriverController.rightBumper().onFalse(m_algaeRollers.stop());

    m_coDriverController
        .leftTrigger()
        .onTrue(m_coralRollers.rollInCommand(CoralLevels.CORAL_STATION));
    m_coDriverController.leftTrigger().onFalse(m_coralRollers.stopCommand());

    m_coDriverController
        .rightTrigger()
        .onTrue(m_coralRollers.rollOutCommand(CoralLevels.CORAL_STATION_OUT));
    m_coDriverController.rightTrigger().onFalse(m_coralRollers.stopCommand());
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
