package frc.robot.constants;

import frc.robot.constants.PoseConstants.Pose;

// https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
public final class ApriltagConstants {
  public enum Blue {
    LEFT_CORAL_STATION(13, new Pose[] {PoseConstants.leftCoralStationPosition2}),
    RIGHT_CORAL_STATION(12, new Pose[] {PoseConstants.rightCoralStationPosition2}),
    PROCESSOR(16, new Pose[] {PoseConstants.prossesor}),
    LEFT_BARGE(14, new Pose[] {PoseConstants.LeftBarge}),
    RIGHT_BARGE(15, new Pose[] {PoseConstants.rightBarge}),
    REEF_1(18, new Pose[] {PoseConstants.ReefPosition1}),
    REEF_2(19, new Pose[] {PoseConstants.ReefPosition2}),
    REEF_3(20, new Pose[] {PoseConstants.ReefPosition3}),
    REEF_4(21, new Pose[] {PoseConstants.ReefPosition4}),
    REEF_5(22, new Pose[] {PoseConstants.ReefPosition5}),
    REEF_6(17, new Pose[] {PoseConstants.ReefPosition6});

    public final int id;
    public final Pose[] pose;

    Blue(final int id, Pose[] pose) {
      this.id = id;
      this.pose = pose;
    }

    public static Blue fromId(final int id) {
      for (Blue e : values()) {
        if (e.id == id) {
          return e;
        }
      }
      return null;
    }

    public Pose[] getPositions() {
      return this.pose;
    }
  }

  public enum Red {
    LEFT_CORAL_STATION(
        1,
        new Pose[] {
          PoseConstants.leftCoralStationPosition2,
          PoseConstants.leftCoralStationPosition1,
          PoseConstants.leftCoralStationPosition3
        }),
    RIGHT_CORAL_STATION(
        2,
        new Pose[] {
          PoseConstants.rightCoralStationPosition2,
          PoseConstants.rightCoralStationPosition1,
          PoseConstants.rightCoralStationPosition3
        }),
    PROCESSOR(3, new Pose[] {PoseConstants.prossesor}),
    LEFT_BARGE(5, new Pose[] {PoseConstants.LeftBarge}),
    RIGHT_BARGE(4, new Pose[] {PoseConstants.rightBarge}),
    REEF_1(7, new Pose[] {PoseConstants.ReefPosition1}),
    REEF_2(6, new Pose[] {PoseConstants.ReefPosition2}),
    REEF_3(11, new Pose[] {PoseConstants.ReefPosition3}),
    REEF_4(10, new Pose[] {PoseConstants.ReefPosition4}),
    REEF_5(9, new Pose[] {PoseConstants.ReefPosition5}),
    REEF_6(8, new Pose[] {PoseConstants.ReefPosition6});

    public final int id;
    public final Pose[] pose;

    Red(final int id, Pose[] pose) {
      this.id = id;
      this.pose = pose;
    }

    public static Red fromId(final int id) {
      for (Red e : values()) {
        if (e.id == id) {
          return e;
        }
      }
      return null;
    }

    public Pose[] getPositions() {
      return this.pose;
    }
  }
}
