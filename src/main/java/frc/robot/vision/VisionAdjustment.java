package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.ApriltagConstants;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionAdjustment {

  public static final String LIMELIGHT_FRONTR = "limelight-frontr";
  public static final String LIMELIGHT_ELEVATOR = "limelight-elevate";

  public static Supplier<String> selectedSideSupplier;

  // in degress
  public static final double verticalTargetFront = -9.84;
  public static final double verticalTargetElevatorLeft = 6.5;
  public static final double verticalTargetElevatorRight = 6.42;

  public static double getTX() {
    return LimelightHelpers.getTX(getNearestLimeLightToTag());
  }

  public static double getTY() {
    return LimelightHelpers.getTY(getNearestLimeLightToTag());
  }

  public static Rotation2d getRotation() {
    int tagID = getClosestApriltag().id;
    Optional<Pose3d> tagPose = ApriltagConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID);
    if (tagPose.isPresent()) {
      return tagPose.get().toPose2d().getRotation();
    } else {
      return null;
    }
  }

  public static double getGoalTX() {

    if (getNearestLimeLightToTag().equals(LIMELIGHT_FRONTR)) {
      return getTY() * 4.73085 + 28.2116;
    } else if (getNearestLimeLightToTag().equals(LIMELIGHT_ELEVATOR)) {
      if (selectedSideSupplier.get().equals("right")) {
        return getTY() * -0.546835 + -3.35932;
      } else if (selectedSideSupplier.get().equals("left")) {
        return getTY() * -0.338092 + 1.8376;
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
      return -1;
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

  public static RawFiducial getClosestApriltag() {
    RawFiducial closestTag = LimelightHelpers.getRawFiducials(getNearestLimeLightToTag())[0];
    for (RawFiducial tag : LimelightHelpers.getRawFiducials(getNearestLimeLightToTag())) {
      if (closestTag.distToCamera < tag.distToCamera) {
        closestTag = tag;
      }
    }
    return closestTag;
  }
}
