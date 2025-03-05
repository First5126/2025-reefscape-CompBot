// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.CoralLevels;
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
          // AprilTagLocalizationConstants.LIMELIGHT_DETAILS_BACKL,
          // AprilTagLocalizationConstants.LIMELIGHT_DETAILS_ELEVATE,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_FRONTR);

  private final LedLights m_ledLights = LedLights.getInstance();
  private final Climbing m_climbing = new Climbing();
  private final AlgaeRollers m_algaeRollers = new AlgaeRollers();
  private final CoralRollers m_coralRollers = new CoralRollers();
  private final Elevator m_elevator = new Elevator();
  private final CoralPivot m_coralPivot = new CoralPivot();
  private final AlgaePivot m_algaePivot =
      new AlgaePivot(m_algaeRollers.hasAlgae(), m_elevator::getCoralLevel);
  private final SendableChooser<Command> autoChooser;
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

    NamedCommands.registerCommand(
        "Go To ReefPose1",
        m_commandFactory.goToPose(PoseConstants.ReefPosition1.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Go To ReefPose2",
        m_commandFactory.goToPose(PoseConstants.ReefPosition2.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Go To ReefPose3",
        m_commandFactory.goToPose(PoseConstants.ReefPosition3.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Go To ReefPose4",
        m_commandFactory.goToPose(PoseConstants.ReefPosition4.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Go To ReefPose5",
        m_commandFactory.goToPose(PoseConstants.ReefPosition5.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Go To ReefPose6",
        m_commandFactory.goToPose(PoseConstants.ReefPosition6.getPose()).asProxy());
    NamedCommands.registerCommand(
        "Raise Elevator to position 1", m_commandFactory.elevatorOutTakeL1());
    NamedCommands.registerCommand(
        "Raise Elevator to position 2", m_commandFactory.elevatorOutTakeL2());
    NamedCommands.registerCommand(
        "Raise Elevator to position 3", m_commandFactory.elevatorOutTakeL3());
    NamedCommands.registerCommand(
        "Raise Elevator to position 4", m_commandFactory.elevatorOutTakeL4());
    NamedCommands.registerCommand(
        "Raise Elevator to position Coral Station",
        m_commandFactory.elevatorInTakeCoralStation().asProxy());

    NamedCommands.registerCommand("Raise Elevator to L4", m_commandFactory.algaeGoToL4().asProxy());
    NamedCommands.registerCommand(
        "Raise Elevator to L3", m_commandFactory.algaeGoToL3().asProxy().withTimeout(2));
    NamedCommands.registerCommand("Process Algae", m_commandFactory.putBallInProcesser().asProxy());
    NamedCommands.registerCommand("Place Coral", m_commandFactory.placeCoral().asProxy());
    NamedCommands.registerCommand(
        "Lower Elevator", m_commandFactory.lowerElevator().asProxy().withTimeout(1.5));
    NamedCommands.registerCommand(
        "Raise Elevator To L2", m_commandFactory.elevatorOutTakeL2().asProxy());

    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    configureCoDriverControls();

    // !!! IMPORTANT !!!
    // Add states disabled to assure state is not going to be destructive when reinabled
    new Trigger(DriverStation::isDisabled)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      System.out.println("DISABLING ROBOT");
                      m_elevator.disable();
                      m_coralPivot.disable();
                      m_algaePivot.disable();
                      m_coralRollers.disable();
                      m_algaeRollers.disable();
                    })
                .ignoringDisable(true));

    // Adds a auto chooser to Shuffle Board to choose autos
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html
    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
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
            m_driverController::getLeftTriggerAxis,
            m_driverController::getRightX,
            m_driverController::getLeftY,
            m_driverController::getLeftX,
            m_elevator::getCoralLevel));

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_driverController.x().whileTrue(m_aprilTagRecognition.getAprilTagCommand());

    m_driverController.a().onTrue(m_aprilTagLocalization.setTrust(true));
    m_driverController.a().onFalse(m_aprilTagLocalization.setTrust(false));

    m_driverController.y().onTrue(m_algaePivot.goToMidPoint());

    m_driverController.start().onTrue(m_commandFactory.zeroRobot());

    m_driverController.b().whileTrue(m_commandFactory.moveBack());
    // Bumpers to coral station

    // d-pad for cardinal movement
    m_driverController.povLeft().whileTrue(m_drivetrain.cardinalMovement(0.025, 0.1));
    m_driverController.povRight().whileTrue(m_drivetrain.cardinalMovement(0.025, -0.1));

    m_driverController.povUp().whileTrue(m_drivetrain.cardinalMovement(0.1, 0));
    m_driverController.povDown().whileTrue(m_drivetrain.cardinalMovement(-0.1, 0));

    // configureDriverAutoCommands();
  }

  private void configureCoDriverControls() {
    // Setup codriver's controls

    m_coDriverController
        .x()
        .whileTrue(m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION));

    m_coDriverController.x().onFalse(m_commandFactory.stopRollers());

    m_coDriverController.back().onTrue(m_algaePivot.goToUpperSetpoint());
    m_coDriverController.y().onTrue(m_commandFactory.algaePivotAndIntake(CoralLevels.DEALGEFY_L3));
    m_coDriverController.a().onTrue(m_commandFactory.algaePivotAndIntake(CoralLevels.PROCESSER));

    m_coDriverController
        .leftTrigger()
        .and(m_coDriverController.leftStick())
        .onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L1));

    // Elevator commands
    m_coDriverController.povUp().and(m_coDriverController.b()).whileTrue(m_elevator.trimUp());
    m_coDriverController.povUp().and(m_coDriverController.b()).onFalse(m_elevator.stopMotors());

    m_coDriverController.povDown().and(m_coDriverController.b()).whileTrue(m_elevator.trimDown());
    m_coDriverController.povDown().and(m_coDriverController.b()).onFalse(m_elevator.stopMotors());

    m_coDriverController
        .povUp()
        .and(this::yIsNotPressed)
        .onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L1));

    m_coDriverController.povRight().onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L2));

    m_coDriverController
        .povDown()
        .and(m_coDriverController.b().negate())
        .onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L3));

    m_coDriverController.povLeft().onTrue(m_commandFactory.coralPivotAndOutake(CoralLevels.L4));

    // algae intake
    m_coDriverController
        .leftBumper()
        .and(m_coDriverController.b().negate())
        .onTrue(m_algaeRollers.feedIn()); // standard
    m_coDriverController
        .leftBumper()
        .and(m_coDriverController.b())
        .onTrue(m_algaeRollers.startFeedIn()); // panic start
    m_coDriverController
        .leftBumper()
        .and(m_coDriverController.b())
        .onFalse(m_algaeRollers.stop()); // panic end

    // algae outake
    m_coDriverController
        .rightBumper()
        .and(m_coDriverController.b().negate())
        .onTrue(m_algaeRollers.feedOut()); // standard
    m_coDriverController
        .rightBumper()
        .and(m_coDriverController.b())
        .onTrue(m_algaeRollers.startFeedOut()); // panic start
    m_coDriverController
        .rightBumper()
        .and(m_coDriverController.b())
        .onFalse(m_algaeRollers.stop()); // panic end

    // coral intake
    m_coDriverController
        .leftTrigger()
        .and(m_coDriverController.b().negate())
        .onTrue(m_coralRollers.rollInCommand(CoralLevels.CORAL_STATION)); // standard
    m_coDriverController.leftTrigger().onFalse(m_coralRollers.stopCommand());

    m_coDriverController
        .leftTrigger()
        .and(m_coDriverController.b())
        .onTrue(m_coralRollers.rollInCommand(CoralLevels.CORAL_STATION)); // panic

    m_coDriverController // standard coral outake
        .rightTrigger()
        .and(m_coDriverController.b().negate())
        .whileTrue(m_commandFactory.coralOutakeAndFlipUp(CoralLevels.CORAL_STATION_OUT));
    m_coDriverController.rightTrigger().onFalse(m_coralRollers.stopCommand());

    m_coDriverController
        .leftTrigger()
        .and(m_coDriverController.b())
        .onTrue(m_coralRollers.rollOutCommand(CoralLevels.L4)); // panic
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
        .andThen(wait);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureDriverAutoCommands() {
    // right bumper left goto

    m_driverController
        .rightBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.hasCoral().negate())
        .whileTrue(
            m_commandFactory
                .moveToPositionWithDistance(
                    PoseConstants.rightCoralStationPositionLeft::getPose,
                    Meters.of(1),
                    m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION))
                .andThen(m_commandFactory.moveBack()));

    // right bumper left return
    m_driverController
        .rightBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.hasCoral())
        .onTrue(m_commandFactory.moveBack());

    // right bumper right goto
    m_driverController
        .rightBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.hasCoral())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.rightCoralStationPositionRight::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // right bumper right return
    m_driverController
        .rightBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.hasCoral().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));

    // left bumper left side goto
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.hasCoral().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.leftCoralStationPositionLeft::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // left bumper left side return
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.hasCoral())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));

    // left bumper right side goto
    m_driverController
        .leftBumper()
        .and(m_recordInputs::rightCoralStationSelected)
        .and(m_coralRollers.hasCoral().negate())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.leftCoralStationPositionRight::getPose,
                Meters.of(1),
                m_commandFactory.coralPivotAndIntake(CoralLevels.CORAL_STATION)));

    // left bumper right side return
    m_driverController
        .leftBumper()
        .and(m_recordInputs::leftCoralStationSelected)
        .and(m_coralRollers.hasCoral())
        .whileTrue(
            m_commandFactory.moveToPositionWithDistance(
                PoseConstants.ReefPosition1::getPose,
                Meters.of(1),
                m_coralPivot.goToUpperSetpoint()));
  }
}
