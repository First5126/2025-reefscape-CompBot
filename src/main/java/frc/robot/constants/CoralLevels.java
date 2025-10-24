package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public enum CoralLevels {
  L1(0.0, 1, -8.0, 5, 0.21, 1.0),
  L2(1.33, 1, -8.0, 3.5, 0.21, 1.0), // changed back to 1.33 from 1.31
  L3(2.75, 1, -8.0, 3.5, 0.21, 1.0),
  L4(4.94, 1, -8.0, 3.5, 0.21, 1.0), // -0.06
  CORAL_STATION(0.78, 1, 7.0, 70.5, 0.21, 1.0),
  CORAL_STATION_OUT(0.78, 1, -5.0, 70.5, 0.21, 1.0),
  TRAVEL(0.5, 1.0, 0.0, 5.0, 0.14, 1.0),
  DEALGEFY_L2(2.40, 1.0, 0.0, 5.0, -0.05, 1.0),
  DEALGEFY_L3(4.17, 1.0, 0.0, 5.0, -0.05, 1.0),
  BARGE(4.17, 1.0, 0.0, 5.0, -0.05, 1.0),
  PROCESSER_TRAVEL(0.722, 1.0, 0.0, 0.0, 0.14, 1.0),
  PROCESSER(0.722, 1.0, 0.0, 0.0, 0.0, 1.0);

  // Height of the elevator expressed in Revolutions.
  public final Angle heightAngle;
  // Distance for starting the raise of the elevator.
  public final Distance distance;
  // Velocity for either intake/outake
  public final Voltage volts;

  public final Angle angle;

  public final Angle algaeAngle;

  // max speed is based on percent 0-1
  public final double maxSpeed;

  private CoralLevels(
      double height,
      double distance,
      Double volts,
      double angle,
      double algaePivot,
      double maxSpeed) {
    this.heightAngle = Revolutions.of(height);
    this.distance = Meters.of(distance);
    this.volts = Volts.of(volts);
    this.angle = Degrees.of(angle);
    this.algaeAngle = Revolutions.of(algaePivot);
    if (maxSpeed > 1) {
      this.maxSpeed = 1.0;
    } else if (maxSpeed < 0) {
      this.maxSpeed = 0.0;
    } else {
      this.maxSpeed = maxSpeed;
    }
  }
}
