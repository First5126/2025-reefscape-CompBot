package frc.robot.constants;

// https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
public final class ApriltagConstants {
  public enum Blue {
    LEFT_CORAL_STATION(13),
    RIGHT_CORAL_STATION(12),
    PROCESSOR(16),
    LEFT_BARGE(14),
    RIGHT_BARGE(15),
    REEF_1(18),
    REEF_2(19),
    REEF_3(20),
    REEF_4(21),
    REEF_5(22),
    REEF_6(17);

    public final int id;

    Blue(final int id){
      this.id = id;
    }
  }
  public enum Red {
    LEFT_CORAL_STATION(1),
    RIGHT_CORAL_STATION(2),
    PROCESSOR(3),
    LEFT_BARGE(5),
    RIGHT_BARGE(4),
    REEF_1(7),
    REEF_2(6),
    REEF_3(11),
    REEF_4(10),
    REEF_5(9),
    REEF_6(8);

    public final int id;

    Red(final int id){
      this.id = id;
    }
  }
}
