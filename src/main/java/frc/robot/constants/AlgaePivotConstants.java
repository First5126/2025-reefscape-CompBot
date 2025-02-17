package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class AlgaePivotConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;  

    public static final double supplyCurrentLimit = 70.0;
    public static final double lowerSupplyCurrentLimit = 10;

    public static final Angle LOWER_ANGLE = Angle.ofBaseUnits(0, Degrees);
    public static final Angle UPPER_ANGLE = Angle.ofBaseUnits(0, Degrees);
}
