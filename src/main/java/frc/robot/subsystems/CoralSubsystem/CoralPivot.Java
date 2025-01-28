package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralPivot extends SubsystemBase {
  private final PWMSparkMax pivotMotor;

  /** Creates a new CoralPivot subsystem. */
  public CoralPivot() {
    pivotMotor = new PWMSparkMax(2); // Assuming motor is connected to PWM port 2
  }

  /**
   * Sets the pivot motor to a specific speed.
   *
   * @param speed The speed to set the motor (range -1.0 to 1.0).
   */
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  /**
   * Stops the pivot motor.
   */
  public void stopPivot() {
    pivotMotor.set(0.0); // Stop the motor
  }

  /**
   * Moves the pivot to a specific position.
   *
   * @param position The target position for the pivot.
   */
  public void moveToPosition(double position) {
    pivotMotor.set(position);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.runOnce(
        () -> {
          setPivotSpeed(0.5); // Example command to set the pivot speed
        },
        this
    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a limit switch).
   *
   * @return value of some boolean subsystem state, such as a limit switch.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a limit switch.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}