package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public enum CoralLevels {
  L1(0.0, 1, -8.0, 5, 0.21),
  L2(1.30, 1, -8.0, 3.5, 0.21),
  L3(2.8163769531249996, 1, -8.0, 3.5, -0.04),
  L4(4.99, 1, -8.0, 3.5, 0.21),
  CORAL_STATION(0.814795, 1, 7.0, 74.5, 0.21),
  CORAL_STATION_OUT(0.914795, 1, -5.0, 74.5, 0.21),
  TRAVEL(0.5, 1.0, 0.0, 5.0, 0.21),
  DEALGEFY_L3(4.17, 1.0, 0.0, 5.0, -0.05),
  PROCESSER(0.722, 1.0, 0.0, 0.0, 0.0);

  // Height of the elevator expressed in Revolutions.
  public final Angle heightAngle;
  // Distance for starting the raise of the elevator.
  public final Distance distance;
  // Velocity for either intake/outake
  public final Voltage volts;

  public final Angle angle;

  public final Angle algaeAngle;

  private CoralLevels(
      double height, double distance, Double volts, double angle, double algaePivot) {
    this.heightAngle = Revolutions.of(height);
    this.distance = Meters.of(distance);
    this.volts = Volts.of(volts);
    this.angle = Degrees.of(angle);
    this.algaeAngle = Revolutions.of(algaePivot);
  }
}
