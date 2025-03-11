package frc.robot.vision;

public class VisonAdjustment {

  public static final String LIMELIGHT_NAME = "limelight-frontr";

  // in degress
  public static final double horizontalTarget = -17.17;
  public static final double verticalTarget = -6;

  public static double getTX() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  public static double getTY() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }
}
