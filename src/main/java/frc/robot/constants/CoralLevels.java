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
  L2(1.33, 1, -8.0, 3.5, 0.21, 0.8),
  L3(2.75, 1, -8.0, 3.5, -0.04, 0.6),
  L4(4.94, 1, -8.0, 3.5, -0.1, 0.4), // -0.06
  CORAL_STATION(0.78, 1, 7.0, 70.5, 0.14, 0.9),
  CORAL_STATION_OUT(0.78, 1, -5.0, 70.5, 0.14, 0.9),
  TRAVEL(0.5, 1.0, 0.0, 5.0, 0.14, 1.0),
  DEALGEFY_L3(4.17, 1.0, 0.0, 5.0, -0.05, 0.6),
  PROCESSER_TRAVEL(0.722, 1.0, 0.0, 0.0, 0.14, 0.9),
  PROCESSER(0.722, 1.0, 0.0, 0.0, 0.0, 0.9);

  // Height of the elevator expressed in Revolutions.
  public final Angle heightAngle;
  // Distance for starting the raise of the elevator.
  public final Distance distance;
  // Velocity for either intake/outake
  public final Voltage volts;
  // Angle of the coral pivot for a given level
  public final Angle coralAngle;
  // Angle of the algae pivot for a given level
  public final Angle algaeAngle;
  // max speed is based on percent 0-1
  public final double maxSpeed;

  private CoralLevels(
      double height,
      double distance,
      Double volts,
      double coralAngle,
      double algaeAngle,
      double maxSpeed) {
    this.heightAngle = Revolutions.of(height);
    this.distance = Meters.of(distance);
    this.volts = Volts.of(volts);
    this.coralAngle = Degrees.of(coralAngle);
    this.algaeAngle = Revolutions.of(algaeAngle);
    if (maxSpeed > 1) {
      this.maxSpeed = 1.0;
    } else if (maxSpeed < 0) {
      this.maxSpeed = 0.0;
    } else {
      this.maxSpeed = maxSpeed;
    }
  }
}
