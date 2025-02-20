package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.ElevatorConstants.CoralLevels;
import frc.robot.constants.PoseConstants.Pose;


public class CommandFactory {
  
  private CommandSwerveDrivetrain m_drivetrain;
  private Supplier<Pose2d> m_robotPoseSupplier;
  private Elevator m_elevator;
  private CoralRollers m_coralRollers;
  private AlgaeRollers m_algaeRollers;
  private Climbing m_climbing;
  private LedLights m_ledLights;
  private CoralPivot m_coralPivot;
  private AlgaePivot m_algaePivot;

  public CommandFactory(
    CommandSwerveDrivetrain drivetrain,
    AlgaeRollers algaeRollers,
    Climbing climbing,
    Elevator elevator,
    CoralRollers coralRollers,
    LedLights ledLights,
    CoralPivot coralPivot,
    AlgaePivot algaePivot
  ) {
    this.m_drivetrain = drivetrain;
    this.m_robotPoseSupplier = m_drivetrain::getPose2d;
    this.m_elevator = elevator;
    this.m_coralRollers = coralRollers;
    this.m_algaeRollers = algaeRollers;
    this.m_climbing = climbing;
    this.m_ledLights = ledLights;
    this.m_coralPivot = coralPivot;
    this.m_algaePivot = algaePivot;
  }

  /*
   * Moves the robot to a position and then runs the secondary commands
   */
  public Command moveToPositionWithDistance(Supplier<Pose2d> position, Distance distance, Supplier<Command> ... secondaryCommands) {
    HashSet<Subsystem> requirements = new HashSet<>();
    requirements.add(m_drivetrain);

    Supplier<Command> gotoPoseSupplier = () -> goToPose(position.get());

    Command goToPose = Commands.defer(gotoPoseSupplier,requirements);
    BooleanSupplier isClose = ()-> position.get().getTranslation().getDistance(m_robotPoseSupplier.get().getTranslation()) <= distance.in(Meters);
    
    // make a new array of commands since Commands.defferedProxy wont accept an array
    Command[] deferedSecondaryCommands = new Command[secondaryCommands.length];
    for (int i = 0; i<secondaryCommands.length;i++) {
      deferedSecondaryCommands[i] = Commands.deferredProxy(secondaryCommands[i]);
    }

    Command then = Commands.waitUntil(isClose).andThen(deferedSecondaryCommands);
    Command returnCommand = Commands.parallel(goToPose,then);
    
    return returnCommand;
  }

  public Command driveAndPlaceCoral(Pose reefPose, CoralLevels level) {
    Command raiseElevator = m_elevator.goToCoralHeightPosition(level);
    Supplier<Command> rasiseElevatorSupplier = () -> raiseElevator;

    Command driveToReef = moveToPositionWithDistance(reefPose::getPose, level.distance, rasiseElevatorSupplier);
    Command placeCoral = m_coralRollers.rollOutCommand();
    Command returnCommand = driveToReef.andThen(placeCoral);

    return returnCommand;
  }

  public Command coralPivotAndIntake() {
    Command pivotCoralRollers = m_coralPivot.goToLowerSetpoint();
    Command intakeCoral = m_coralRollers.rollInCommand();
    Command finishIntake = m_coralPivot.goToUpperSetpoint().alongWith(m_coralRollers.stopCommand());

    return pivotCoralRollers.alongWith(intakeCoral).until(m_coralRollers.getCoralTrigger()).andThen(finishIntake);
  }

  public Command algaePivotAndIntake() {
    Command pivotAlgaeRollers = m_algaePivot.goToLowerSetpoint();
    Command intakeAlgae = m_algaeRollers.feedIn();

    Command finishIntake = m_algaePivot.goToUpperSetpoint().alongWith(m_algaeRollers.stop());
    return pivotAlgaeRollers.alongWith(intakeAlgae).until(m_algaeRollers.hasGamePiece()).andThen(finishIntake);
  }

  public Command goToPose(Pose2d pose) {
    return m_drivetrain.goToPose(pose);
  }
}
