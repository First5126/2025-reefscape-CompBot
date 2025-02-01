package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRoller extends SubsystemBase {
  private final PWMSparkMax rollerMotor;

  /** Creates a new AlgaeRoller subsystem. */
  public AlgaeRoller() {
    rollerMotor = new PWMSparkMax(4); // Assuming motor is connected to PWM port 4
  }

  /**
   * Starts the roller mechanism.
   */
  public void startRoller() {
    rollerMotor.set(1.0); // Set motor speed to full power
  }

  /**
   * Reverses the roller mechanism.
   */
  public void reverseRoller() {
    rollerMotor.set(-1.0); // Set motor speed to reverse full power
  }

  /**
   * Stops the roller mechanism.
   */
  public void stopRoller() {
    rollerMotor.set(0.0); // Stop the motor
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
          startRoller();
        },
        this
    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
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