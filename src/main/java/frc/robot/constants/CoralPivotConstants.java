package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class CoralPivotConstants {
  public static final double MotionMagicAcceleration = 18;
  // public static final double MotionMagicAcceleration = 180;
  // public static final double MotionMagicCruiseVelocity = 4;
  public static final double MotionMagicCruiseVelocity = 0.1;
  public static final double MotionMagicJerk = 1800;

  public static final double kP = 90;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final double supplyCurrentLimit = 70.0;
  public static final double lowerSupplyCurrentLimit = 10;

  public static final Angle LOWER_ANGLE = Degrees.of(180);
  public static final Angle UPPER_ANGLE = Degrees.of(0);
  public static final Angle CORAL_STATION_ANGLE = Degrees.of(168.12);
}
