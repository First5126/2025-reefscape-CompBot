package frc.robot.vision;

public class VisonAdjustment {

  public static final String LIMELIGHT_NAME = "limelight-frontr";

  // in degress
  public static final double horizontalTarget = -16.45;
  public static final double verticalTarget = -6.1;
  public static final double minimumVertialTarget = -7.0;

  public static double getTX() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  public static double getTY() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }

  public static double getGoalTX() {
    return getTY() * 6.70558 + 23.9141;
  }

  public static boolean hasTarget() {
    return LimelightHelpers.getRawFiducials(LIMELIGHT_NAME).length > 0;
  }

  public static boolean shouldStrafe() {
    return LimelightHelpers.getRawFiducials(LIMELIGHT_NAME).length > 0
        && verticalTarget > minimumVertialTarget;
  }
}
