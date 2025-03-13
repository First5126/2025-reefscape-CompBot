package frc.robot.vision;

import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.function.Supplier;

public class VisonAdjustment {

  public static final String LIMELIGHT_FRONTR = "limelight-frontr";
  public static final String LIMELIGHT_ELEVATOR = "limelight-elevate";

  public static Supplier<String> selectedSideSupplier;

  // in degress
  public static final double verticalTarget_front = -6.1;
  public static final double verticalTarget_elevator = 5.18;

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

  public static double getTX() {
    return LimelightHelpers.getTX(getNearestLimeLightToTag());
  }

  public static double getTY() {
    return LimelightHelpers.getTY(getNearestLimeLightToTag());
  }

  public static double getGoalTX() {

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return getTY() * 6.70558 + 23.9141;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      if (selectedSideSupplier.get().equals("right")) {
        return getTY() * -0.529032 + -6.77097;
      } else if (selectedSideSupplier.get().equals("left")) {
        return getTY() * -0.410133 + 0.00301568;
      }
    }
    return getTY();
  }

  public static double getGoalTY() {

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return verticalTarget_front;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      return verticalTarget_elevator;
    }
    return getTY();
  }

  public static int getInversion() {
    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return -1;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      return 1;
    }
    return 1;
  }

  public static boolean hasTarget() {
    return LimelightHelpers.getRawFiducials(getNearestLimeLightToTag()).length > 0;
  }
}
