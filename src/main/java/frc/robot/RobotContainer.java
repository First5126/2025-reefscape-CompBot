// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.TriggerEvent;

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
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS);

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
// as Proxy
    NamedCommands.registerCommand("Place Coral at L4", m_elevator.setCoralPosition(CoralLevels.L4).asProxy());// These commands are registared to Shuffleboard.
    NamedCommands.registerCommand("Place Coral at L3", m_elevator.setCoralPosition(CoralLevels.L3).asProxy());
    NamedCommands.registerCommand("Place Coral at L2", m_elevator.setCoralPosition(CoralLevels.L2).asProxy());
    NamedCommands.registerCommand("Place Coral at L1", m_elevator.setCoralPosition(CoralLevels.L1).asProxy());
    NamedCommands.registerCommand("Get Coral From Station 1", m_coralRollers.rollInCommand().asProxy());

    new EventTrigger("L4").onTrue(Commands.run(() -> System.out.println("L4")));
    new EventTrigger("L3").onTrue(Commands.run(() -> System.out.println("L3")));
    new EventTrigger("L2").onTrue(Commands.run(() -> System.out.println("L2")));
    new EventTrigger("L1").onTrue(Commands.run(() -> System.out.println("L1")));

    configureBindings();
    configureCoDriverControls();

    // Adds a auto chooser to Shuffle Board to choose autos
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Place Coral at L4", m_elevator.setCoralPosition(CoralLevels.L4));
  }

  private boolean yIsNotPressed() {
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
            m_driverController::getLeftTriggerAxis,
            m_driverController::getRightX,
            m_driverController::getLeftY,
            m_driverController::getLeftX));

    m_driverController.povUp().and(this::yIsNotPressed).onTrue(m_elevator.raiseToNextPosition());
    m_driverController.povDown().and(this::yIsNotPressed).onTrue(m_elevator.lowerToNextPosition());
    m_driverController.povUp().and(this::yIsPressed).onTrue(m_elevator.trimUp());
    m_driverController.povDown().and(this::yIsPressed).onTrue(m_elevator.trimDown());

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.x().whileTrue(m_aprilTagRecognition.getAprilTagCommand());
    // m_driverController.x().onTrue(m_coralPivot.goToLowerSetpoint());
    // m_driverController.y().onTrue(m_coralPivot.goToUpperSetpoint());
  }

  private void configureCoDriverControls() {
    // Setup codriver's controlls
    m_coDriverController
        .a()
        .whileTrue(m_coralRollers.rollInCommand())
        .onFalse(m_coralRollers.stopCommand());
    m_coDriverController
        .b()
        .whileTrue(
            m_coralRollers.rollOutCommand(
                CoralLevels.L2)) // TODO: This should be coming from a state of selection.
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
    return autoChooser.getSelected();
  }
}
