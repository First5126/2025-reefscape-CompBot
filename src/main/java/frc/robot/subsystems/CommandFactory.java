package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.CoralLevels;
import frc.robot.constants.PoseConstants.Pose;
import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_robotPoseSupplier;
  private Elevator m_elevator;
  private CoralRollers m_coralRollers;
  private AlgaeRollers m_algaeRollers;
  private Climbing m_climbing;
  //private LedLights m_ledLights;
  private CoralPivot m_coralPivot;
  private AlgaePivot m_algaePivot;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      AlgaeRollers algaeRollers,
      Climbing climbing,
      Elevator elevator,
      CoralRollers coralRollers,
      CoralPivot coralPivot,
      AlgaePivot algaePivot) {
    this.m_drivetrain = drivetrain;
    this.m_robotPoseSupplier = m_drivetrain::getPose2d;
    this.m_elevator = elevator;
    this.m_coralRollers = coralRollers;
    this.m_algaeRollers = algaeRollers;
    this.m_climbing = climbing;
    this.m_coralPivot = coralPivot;
    this.m_algaePivot = algaePivot;
  }

  /*
   * Moves the robot to a position and then runs the secondary commands
   */
  public Command moveToPositionWithDistance(
      Supplier<Pose2d> position, Distance distance, Command... secondaryCommands) {
    HashSet<Subsystem> requirements = new HashSet<>();
    requirements.add(m_drivetrain);

    Supplier<Command> gotoPoseSupplier = () -> goToPose(position.get());

    Command goToPose = Commands.defer(gotoPoseSupplier, requirements);
    BooleanSupplier isClose =
        () ->
            position.get().getTranslation().getDistance(m_robotPoseSupplier.get().getTranslation())
                <= distance.in(Meters);

    Command then = Commands.waitUntil(isClose).andThen(secondaryCommands);
    Command returnCommand = Commands.deferredProxy(() -> Commands.parallel(goToPose, then));

    return returnCommand;
  }

  public Command driveAndPlaceCoral(Pose reefPose, CoralLevels level) {
    Command raiseElevator = m_elevator.setCoralPosition(level);
    Command driveToReef =
        moveToPositionWithDistance(reefPose::getPose, level.distance, raiseElevator);
    Command placeCoral = m_coralRollers.rollOutCommand(level);
    Command returnCommand = driveToReef.andThen(placeCoral);

    return returnCommand;
  }

  public Command stopRollers() {
    return m_coralRollers.stopCommand();
  }

  public Command coralPivotAndIntake(CoralLevels level) {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.CORAL_STATION);
    Command pivotCoralRollers = m_coralPivot.gotoCoralStationSetpoint();
    Command pivotAlgaeRollers = m_algaePivot.goToLevel(level);
    Command intakeCoral = m_coralRollers.rollInCommand(level);
    Command finishIntake = m_coralPivot.goToUpperSetpoint().alongWith(m_coralRollers.stopCommand());

    return elevator
        .andThen(pivotCoralRollers)
        .alongWith(intakeCoral)
        .alongWith(pivotAlgaeRollers)
        .until(m_coralRollers.hasCoral())
        /*.andThen(
        Commands.deadline(
            Commands.waitSeconds(.3), m_drivetrain.cardinalMovement(-.25, 0).asProxy()))*/
        .andThen(finishIntake);
    // .andThen(m_elevator.setCoralPosition(CoralLevels.TRAVEL));
  }

  public Command coralPivotAndOutake(CoralLevels level) {
    Command elevator = m_elevator.setCoralPosition(level);
    Command pivotCoralRollers = m_coralPivot.gotoAngle(level.angle);
    Command pivotAlgaeRolllers = m_algaePivot.setAngle(level);

    return elevator.andThen(pivotCoralRollers).alongWith(pivotAlgaeRolllers);
  }

  public Command algaePivotAndOutake(CoralLevels level) {
    Command pivotAlgaeRollers = m_algaePivot.goToLevel(level);
    Command elevator = m_elevator.setCoralPosition(level);
    Command pivotCoralRollers = m_coralPivot.gotoAngle(level.angle);
    Command finalCommand = pivotCoralRollers.alongWith(pivotAlgaeRollers).andThen(elevator);

    return finalCommand;
  }

  public Command goToPose(Pose2d pose) {
    return m_drivetrain.goToPose(pose);
  }

  public Command moveBack() {
    Rotation2d rotation = m_robotPoseSupplier.get().getRotation();
    double x = m_robotPoseSupplier.get().getX() - (0.3 * Math.cos(rotation.getDegrees()));
    double y = m_robotPoseSupplier.get().getY() - (0.3 * Math.sin(rotation.getDegrees()));
    double[] debugArray = {x, y};

    SmartDashboard.putNumberArray("Move Back Position", debugArray);

    double[] debugArray2 = {
      0.3 * Math.cos(rotation.getDegrees()), 0.3 * Math.sin(rotation.getDegrees())
    };
    SmartDashboard.putNumberArray("Signs", debugArray2);

    return goToPose(new Pose2d(x, y, rotation));
  }

  public Command elevatorOutTakeL1() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L1);
    Command pivotCoralRollers = m_coralPivot.goToLowerSetpoint();
    Command outTakeCoral = m_coralRollers.rollOutCommand(CoralLevels.L1);

    return elevator.andThen(pivotCoralRollers).alongWith(outTakeCoral);
  }

  public Command elevatorOutTakeL2() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L2);
    Command pivotCoralRollers = m_coralPivot.goToLowerSetpoint();
    Command outTakeCoral = m_coralRollers.rollOutCommand(CoralLevels.L2);

    return elevator.andThen(pivotCoralRollers).alongWith(outTakeCoral);
  }

  public Command elevatorOutTakeL3() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L3);
    Command pivotCoralRollers = m_coralPivot.goToLowerSetpoint();
    Command outTakeCoral = m_coralRollers.rollOutCommand(CoralLevels.L3);

    return elevator.andThen(pivotCoralRollers).alongWith(outTakeCoral);
  }

  public Command elevatorOutTakeL4() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L4);
    Command pivotCoralRollers = m_coralPivot.goToLowerSetpoint();
    Command outTakeCoral = m_coralRollers.rollOutCommand(CoralLevels.L4);

    return elevator.andThen(pivotCoralRollers).alongWith(outTakeCoral);
  }

  public Command elevatorInTakeCoralStation() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.CORAL_STATION);
    Command pivotCoralRollers = m_coralPivot.goToUpperSetpoint();
    Command inTakeCoral = m_coralRollers.rollInCommand(CoralLevels.CORAL_STATION);

    return elevator.andThen(pivotCoralRollers).alongWith(inTakeCoral);
  }

  public Command coralOutakeAndFlipUp(CoralLevels level) {
    return m_coralRollers
        .rollOutCommand(level)
        .andThen(Commands.waitSeconds(0.1))
        .andThen(m_coralPivot.goToUpperSetpoint())
        .andThen(
            Commands.deadline(Commands.waitSeconds(.3), m_drivetrain.cardinalMovement(-.25, 0)))
        .andThen(m_elevator.setCoralPosition(CoralLevels.L3));
  }

  public Command zeroRobot() {
    return m_drivetrain.zero_pidgeon().alongWith(m_elevator.zeroElevator());
  }

  public Command algaeGoToL3() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L3);
    Command pivotAlgaeRollers = m_algaePivot.goToLowerSetpoint();
    Command IntakeAlgae = m_algaeRollers.feedIn();

    return elevator
        .andThen(pivotAlgaeRollers)
        .alongWith(IntakeAlgae)
        .until(m_algaeRollers.hasAlgae());
  }

  public Command dealegfyL3() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.DEALGEFY_L3);
    Command pivotAlgaeRollers = m_algaePivot.goToLowerSetpoint();
    Command IntakeAlgae = m_algaeRollers.feedIn();

    return elevator.andThen(pivotAlgaeRollers).alongWith(IntakeAlgae);
  }

  public Command dealegfyL2() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.DEALGEFY_L2).withTimeout(8);
    Command pivotAlgaeRollers = m_algaePivot.goToLowerSetpoint();
    Command IntakeAlgae = m_algaeRollers.feedIn();

    return elevator.andThen(pivotAlgaeRollers).alongWith(IntakeAlgae);
  }

  public Command putBallInProcesser() {
    Command goToLevelProcesser = m_elevator.setCoralPosition(CoralLevels.PROCESSER_TRAVEL);
    Command finalCommand =
        goToLevelProcesser.withTimeout(0.5).andThen(m_algaeRollers.startFeedOut());

    return finalCommand;
  }

  public Command placeCoralL4() {
    return placeCoralonL4();
  }

  public Command placeCoralL3() {
    return placeCoralonL3();
  }

  public Command placeCoralL2() {
    return placeCoralonL2();
  }

  public Command lowerElevator() {
    Command lowerL3Elevator = m_elevator.goToBottom();

    return lowerL3Elevator;
  }

  private Command placeCoralonL4() {
    System.out.println("Placing Coral L4");
    Command ReleaseCoral = m_coralRollers.rollOutCommand(CoralLevels.L4);

    SmartDashboard.putData("Place Coral L4", ReleaseCoral);
    return ReleaseCoral;
  }

  private Command placeCoralonL3() {
    Command pivotCoralRollersCommand = m_coralPivot.goToLowerSetpoint();
    Command ReleaseCoral = m_coralRollers.rollOutCommand(CoralLevels.L3);
    Command algaePivot = m_algaePivot.setAngle(CoralLevels.L3);

    return algaePivot.andThen(pivotCoralRollersCommand).withTimeout(0.1).andThen(ReleaseCoral);
  }

  private Command placeCoralonL2() {
    Command pivotCoralRollersCommand = m_coralPivot.goToLowerSetpoint();
    Command ReleaseCoral = m_coralRollers.rollOutCommand(CoralLevels.L2);
    Command algaePivot = m_algaePivot.setAngle(CoralLevels.L2);

    SmartDashboard.putData("Place Coral L2", ReleaseCoral);
    return algaePivot.andThen(pivotCoralRollersCommand).withTimeout(0.1).andThen(ReleaseCoral);
  }

  public Command intakeCoral() {
    Command coralIntake = m_coralRollers.rollInCommand(CoralLevels.CORAL_STATION);
    SmartDashboard.putData("Intaking Coral", coralIntake);

    return coralIntake;
  }

  public Command moveElevatorUpToL2() {
    Command elevator =
        m_elevator.setCoralPosition(CoralLevels.L2).until(m_elevator::elevatorAtPosition);
    Command pivotCoralRollersCommand = m_coralPivot.goToLowerSetpoint();

    return elevator.andThen(pivotCoralRollersCommand);
  }

  public Command moveElevatorUpToCoralStation() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.CORAL_STATION);
    Command algaePivot = m_algaePivot.setAngle(CoralLevels.CORAL_STATION);
    Command coralPiviot = m_coralPivot.gotoCoralStationSetpoint();

    return elevator.alongWith(coralPiviot).alongWith(algaePivot);
  }

  public Command moveElevatorUpToL3() {
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L3);

    return elevator;
  }

  public Command moveElevatorUpToL4() {
    Command pivotCoralRollersCommand = m_coralPivot.goToLowerSetpoint();
    Command elevator = m_elevator.setCoralPosition(CoralLevels.L4);
    Command algaePivot = m_algaePivot.setAngle(CoralLevels.L4);

    return elevator.alongWith(algaePivot).alongWith(pivotCoralRollersCommand);
  }

  public Command algaePivotAndIntake(CoralLevels level) {
    Command elevator = m_elevator.setCoralPosition(level);
    Command pivotCoralRollers = m_coralPivot.gotoAngle(level.angle);
    Command pivotAlgaeRolllers = m_algaePivot.goToLevel(level);

    return elevator.andThen(pivotCoralRollers).alongWith(pivotAlgaeRolllers);
  }

  /**
   * This command will set the level for algae processing travel.
   *
   * @return A command for setting the algae procession level for traveling only
   */
  public Command setAlgaeProcessorLevel() {
    CoralLevels level = CoralLevels.PROCESSER_TRAVEL;

    Command elevator = m_elevator.setCoralPosition(level);
    Command algaePivot = m_algaePivot.setAngle(level);
    Command coralPivot = m_coralPivot.gotoAngle(level.angle);
    return elevator.alongWith(algaePivot).alongWith(coralPivot);
  }

  /**
   * This command will process algae in the processor. It will first lower the angle to a shooting
   * level and then feed out the ball.
   *
   * @return A command for processing the algae in the side processor
   */
  public Command processAlgae() {
    CoralLevels level = CoralLevels.PROCESSER;

    Command elevator = m_elevator.setCoralPosition(level);
    Command algaePivot = m_algaePivot.setAngle(level);
    Command runWheels = m_algaeRollers.startFeedIn();
    Command coralPivot = m_coralPivot.gotoAngle(level.angle);
    Command algaeFeedOut = m_algaeRollers.startFeedOut();

    return elevator
        .alongWith(algaePivot)
        .andThen(coralPivot)
        .alongWith(runWheels)
        .andThen(Commands.waitSeconds(0.3))
        .andThen(algaeFeedOut);
  }

  public Command waitUntilCoralIn() {
    Command runIntake = elevatorInTakeCoralStation();

    return runIntake.until(m_coralRollers.hasCoral());
  }
}
