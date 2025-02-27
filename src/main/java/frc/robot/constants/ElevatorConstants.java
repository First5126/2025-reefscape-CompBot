package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class ElevatorConstants {
  public static enum CoralLevels {
    L1(0.0, 1),
    L2(1.35009765625, 1),
    L3(2.8163769531249996, 1),
    L4(4.975830078125, 1);

    // Height of the elevator expressed in Revolutions.
    public final Angle heightAngle;
    // Distance for starting the raise of the elevator.
    public final Distance distance;

    private CoralLevels(double height, double distance) {
      this.heightAngle = Revolutions.of(height);
      this.distance = Meters.of(distance);
    }
  }

  public static final int ELEVATOR_MOTOR_PORT = 1;
  public static final double ELEVATOR_SPEED = 0.75;
  public static final double ELEVATOR_MAX_HEIGHT = 2.0; // in meters
  public static final double ELEVATOR_MIN_HEIGHT = 0.0; // in meters
  public static final double ELEVATOR_TOLERANCE = 0.01; // in meters

  public static final double kP = 60;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kV = 0;
  public static final double kS = 0.16;
  public static final double kG = 0.032;

  public static final int FORWARD_DIGITAL_LIMIT = 0;
  public static final int REVERSE_DIGITAL_LIMIT = 0;

  public static final double GEAR_RATIO = 12.0;

  public static final Angle ELEVATOR_READING_STDV = Revolutions.of(0.05);

  public static final Supplier<Double> TRIM_UP_SPEED = () -> Double.valueOf(1);
  public static final Supplier<Double> TRIM_DOWN_SPEED = () -> Double.valueOf(-1);
}
