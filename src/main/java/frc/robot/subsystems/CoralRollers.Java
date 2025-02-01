package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRollers extends SubsystemBase {
  private final PWMSparkMax intakeMotor;
  private final PWMSparkMax coralIntakeMotor;

  /** Creates a new CoralRollers subsystem. */
  public CoralRollers() {
    intakeMotor = new PWMSparkMax(0); // Assuming motor is connected to PWM port 0
    coralIntakeMotor = new PWMSparkMax(1); // Assuming coral intake motor is connected to PWM port 1
  }

  /**
   * Starts the intake mechanism.
   */
  public void startIntake() {
    intakeMotor.set(1.0); // Set motor speed to full power
  }

  /**
   * Stops the intake mechanism.
   */
  public void stopIntake() {
    intakeMotor.set(0.0); // Stop the motor
  }

  /**
   * Starts the coral intake mechanism.
   */
  public void startCoralIntake() {
    coralIntakeMotor.set(1.0); // Set coral intake motor speed to full power
  }

  /**
   * Stops the coral intake mechanism.
   */
  public void stopCoralIntake() {
    coralIntakeMotor.set(0.0); // Stop the coral intake motor
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
          startIntake();
          startCoralIntake();
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