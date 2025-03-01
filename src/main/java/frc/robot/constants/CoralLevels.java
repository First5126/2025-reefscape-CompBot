package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public enum CoralLevels {
  L1(0.0, 1, -100, 5),
  L2(1.30, 1, -110, 3.5),
  L3(2.8163769531249996, 1, -110, 3.5),
  L4(4.975830078125, 1, -110, 3.5),
  CORAL_STATION(0.914795, 1, 80, 74.5),
  CORAL_STATION_OUT(0.914795, 1, -110, 74.5);

  // Height of the elevator expressed in Revolutions.
  public final Angle heightAngle;
  // Distance for starting the raise of the elevator.
  public final Distance distance;
  // Velocity for either intake/outake
  public final AngularVelocity velocity;

  public final Angle angle;

  private CoralLevels(double height, double distance, int revolutionsPerSecond, double angle) {
    this.heightAngle = Revolutions.of(height);
    this.distance = Meters.of(distance);
    this.velocity = RevolutionsPerSecond.of(revolutionsPerSecond);
    this.angle = Degrees.of(angle);
  }
}
