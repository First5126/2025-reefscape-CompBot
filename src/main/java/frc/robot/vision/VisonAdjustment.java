package frc.robot.vision;

import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.function.Supplier;

public class VisonAdjustment {

  public static final String LIMELIGHT_FRONTR = "limelight-frontr";
  public static final String LIMELIGHT_ELEVATOR = "limelight-elevate";

  public static Supplier<String> selectedSideSupplier;

  // in degress
  public static final double verticalTargetFront = 9.47;
  public static final double verticalTargetElevatorLeft = 8.75;
  public static final double verticalTargetElevatorRight = 8.72;
  public static final double verticalTargetElevatorProcessor = 0.0;

  public static final int[] coralStationIDs = {1, 2, 12, 13};
  public static final int[] processerIDs = {3, 16};

  public static double getTX() {
    return LimelightHelpers.getTX(getNearestLimeLightToTag());
  }

  public static double getTY() {
    return LimelightHelpers.getTY(getNearestLimeLightToTag());
  }

  public static RawFiducial getNearestTag() {
    double nearestTagDistance = Double.POSITIVE_INFINITY;
    RawFiducial nearestTag = null;

    for (RawFiducial tag : LimelightHelpers.getRawFiducials(getNearestLimeLightToTag())) {
      if (tag.distToCamera < nearestTagDistance) {
        nearestTag = tag;
        nearestTagDistance = tag.distToCamera;
      }
    }

    return nearestTag;
  }

  public static double getGoalTX() {

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return getTY() * 7.68525 + -58.9293;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      if (selectedSideSupplier.get().equals("right")) {
        return getTY() * -0.393498 + -2.7087;
      } else if (selectedSideSupplier.get().equals("left")) {
        return getTY() * -0.323929 + 3.49438;
      }
    }
    return getTY();
  }

  public static double getGoalTY() {

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return verticalTargetFront;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      if (selectedSideSupplier.get().equals("right")) {
        return verticalTargetElevatorRight;
      } else if (selectedSideSupplier.get().equals("left")) {
        return verticalTargetElevatorLeft;
      }
    }
    return getTY();
  }

  public static int getInversion() {
    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return 1;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      return 1;
    }
    return 1;
  }

  public static boolean hasTarget() {
    return LimelightHelpers.getRawFiducials(getNearestLimeLightToTag()).length > 0;
  }

  private static String getNearestLimeLightToTag() {
    double nearestTagDistance = Double.POSITIVE_INFINITY;
    String nearestLimelight = "";

    for (RawFiducial tag : LimelightHelpers.getRawFiducials(LIMELIGHT_FRONTR)) {
      if (nearestTagDistance > tag.distToCamera) {
        nearestTagDistance = tag.distToCamera;
        nearestLimelight = LIMELIGHT_FRONTR;
      }
    }

    for (RawFiducial tag : LimelightHelpers.getRawFiducials(LIMELIGHT_ELEVATOR)) {
      if (nearestTagDistance > tag.distToCamera) {
        nearestTagDistance = tag.distToCamera;
        nearestLimelight = LIMELIGHT_ELEVATOR;
      }
    }

    return nearestLimelight;
  }
}
