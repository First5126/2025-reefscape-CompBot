package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.function.Supplier;

public class VisonAdjustment {

  public static final String LIMELIGHT_FRONT = "limelight-front";
  public static final String LIMELIGHT_ELEVATOR = "limelight-elevate";

  public static Supplier<String> selectedSideSupplier;

  // in degress
  public static final double verticalTargetFront = 9.47;
  public static final double verticalTargetElevatorLeft = 8.75;
  public static final double verticalTargetElevatorRight = 8.72;
  public static final double verticalTargetElevatorProcessor = -7.53;

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

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONT)) {
      return 16.66;
      //return getTY() * 7.68525 + -57.1;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      if (lookingAt(coralStationIDs)) {
        if (selectedSideSupplier.get().equals("right")) {
          return -4.80;
          //return getTY() * -0.393498 + -2.7087;
        } else if (selectedSideSupplier.get().equals("left")) {
          return 1.93;
          //return getTY() * -0.323929 + 3.49438;
        }
      } else if (lookingAt(processerIDs)) {
        return getTY() * -0.636596 + -8.28357;
      }
    }
    return getTY();
  }

  public static double getGoalTY() {
    SmartDashboard.putBoolean("LOOKING AT PROCESSER", lookingAt(processerIDs));
    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONT)) {
      return verticalTargetFront;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {

      if (lookingAt(coralStationIDs)) {
        if (selectedSideSupplier.get().equals("right")) {
          return verticalTargetElevatorRight;
        } else if (selectedSideSupplier.get().equals("left")) {
          return verticalTargetElevatorLeft;
        }
      } else if (lookingAt(processerIDs)) {
        return verticalTargetElevatorProcessor;
      }
    }
    return getTY();
  }

  public static int getInversion() {
    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONT)) {
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

    for (RawFiducial tag : LimelightHelpers.getRawFiducials(LIMELIGHT_FRONT)) {
      if (nearestTagDistance > tag.distToCamera) {
        nearestTagDistance = tag.distToCamera;
        nearestLimelight = LIMELIGHT_FRONT;
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

  private static boolean lookingAt(int[] list) {
    boolean found = false;
    int id = getNearestTag().id;

    for (int otherId : list) {
      if (otherId == id) {
        found = true;
        break;
      }
    }

    return found;
  }
}
