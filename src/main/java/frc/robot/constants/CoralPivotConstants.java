package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class CoralPivotConstants {
  public static final double MotionMagicAcceleration = 32;
  public static final double MotionMagicCruiseVelocity = 4;
  public static final double MotionMagicJerk = 320;

  public static final double kP = 45;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final double supplyCurrentLimit = 70.0;
  public static final double lowerSupplyCurrentLimit = 10;

  public static final Angle LOWER_ANGLE = Degrees.of(0);
  public static final Angle UPPER_ANGLE = Degrees.of(100);
  public static final Angle CORAL_STATION_ANGLE = Degrees.of(74.5);

  public static final Angle L1 = Degrees.of(140);
  public static final Angle L2 = Degrees.of(98.6);
  public static final Angle L3 = Degrees.of(98.6);
  public static final Angle L4 = Degrees.of(98.6);
}
