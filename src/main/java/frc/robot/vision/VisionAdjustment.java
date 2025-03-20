package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
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
    SmartDashboard.putString("LimeLight Rotation Reading", "Getting Rotation");
    if (getClosestApriltag() == null) {
      SmartDashboard.putString("LimeLight Rotation Reading", "Tag is null");
      return null;
    }
    int tagID = getClosestApriltag().id;
    SmartDashboard.putString("LimeLight Rotation Reading", "End of getting closestApriltagId");
    Optional<Pose3d> tagPose = RobotContainer.FIELD_LAYOUT.getTagPose(tagID);
    SmartDashboard.putString("LimeLight Rotation Reading", "End of tagPose Optional");
    if (tagPose.isPresent()) {
      SmartDashboard.putString("LimeLight Rotation Reading", "Tag Pressent");
      return tagPose.get().toPose2d().getRotation();
    } else {
      SmartDashboard.putString("LimeLight Rotation Reading", "Tag Null");
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
    RawFiducial closestTag;
    if (LimelightHelpers.getRawFiducials(getNearestLimeLightToTag()).length != 0) {
      closestTag = LimelightHelpers.getRawFiducials(getNearestLimeLightToTag())[0];
    } else {
      return null;
    }
    for (RawFiducial tag : LimelightHelpers.getRawFiducials(getNearestLimeLightToTag())) {
      if (closestTag.distToCamera < tag.distToCamera) {
        closestTag = tag;
      }
    }
    return closestTag;
  }
}
