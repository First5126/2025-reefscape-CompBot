package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseConstants {

  public static class Pose {
    private Pose2d redPose, bluePose;

    public Pose(Pose2d bluePose, Pose2d redPose) {
      this.bluePose = bluePose;
      this.redPose = redPose;
    }

    public Pose2d getPose() {
      if (DriverStation.getAlliance().get().name().equals("red")) {
        return redPose;
      } else {
        return bluePose;
      }
    }
  }

  // TODO: None of these positions are true. They were all in testing and will NOT WORK AT ALL.

  // tag numbers 1 and 13
  public static final Pose leftCoralStationPositionRight =
      new Pose(
          new Pose2d(1.7, 7.3, Rotation2d.fromDegrees(125)),
          new Pose2d(15.927, 0.731, Rotation2d.fromDegrees(-55)));
  public static final Pose leftCoralStationPosition2 =
      new Pose(
          new Pose2d(1.3, 7, Rotation2d.fromDegrees(125)),
          new Pose2d(16.39, 1078, Rotation2d.fromDegrees(-55)));
  public static final Pose leftCoralStationPositionLeft =
      new Pose(
          new Pose2d(0.8, 6.6, Rotation2d.fromDegrees(125)),
          new Pose2d(16.789, 1.338, Rotation2d.fromDegrees(-55)));

  // tag numbers 2 and 12
  public static final Pose rightCoralStationPositionRight =
      new Pose(
          new Pose2d(1.7, 0.8, Rotation2d.fromDegrees(-125)),
          new Pose2d(16.853, 6.619, Rotation2d.fromDegrees(55)));
  public static final Pose rightCoralStationPosition2 =
      new Pose(
          new Pose2d(1.3, 1, Rotation2d.fromDegrees(-125)),
          new Pose2d(16.405, 6.922, Rotation2d.fromDegrees(55)));
  public static final Pose rightCoralStationPositionLeft =
      new Pose(
          new Pose2d(0.8, 1.4, Rotation2d.fromDegrees(-125)),
          new Pose2d(15.927, 7.284, Rotation2d.fromDegrees(55)));

  // tag numbers 3 and 16
  public static final Pose prossesor =
      new Pose(
          new Pose2d(6.4, 0.6, Rotation2d.fromDegrees(-90)),
          new Pose2d(11.513, 7.448, Rotation2d.fromDegrees(90)));

  // tag numbers 4 and 15
  public static final Pose rightBarge =
      new Pose(
          new Pose2d(7.535, 5.064, Rotation2d.fromDegrees(0)),
          new Pose2d(9.994, 3.021, Rotation2d.fromDegrees(180)));

  // tag numbers no clue
  public static final Pose middleBarge =
      new Pose(
          new Pose2d(7.535, 6.17, Rotation2d.fromDegrees(0)),
          new Pose2d(9.965, 1.922, Rotation2d.fromDegrees(180)));

  // tag numbers 5 and 14
  public static final Pose LeftBarge =
      new Pose(
          new Pose2d(7.535, 7.225, Rotation2d.fromDegrees(0)),
          new Pose2d(10.009, 0.822, Rotation2d.fromDegrees(180)));

  // tag numbers 10 and 18
  public static final Pose ReefPosition1 =
      new Pose(
          new Pose2d(3.130, 4.008, Rotation2d.fromDegrees(0)),
          new Pose2d(14.22, 4.011, Rotation2d.fromDegrees(0)));

  // tag numbers 9 and 19
  public static final Pose ReefPosition2 =
      new Pose(
          new Pose2d(3.810, 5.195, Rotation2d.fromDegrees(-60)),
          new Pose2d(13.745, 2.831, Rotation2d.fromDegrees(120)));

  // tag numbers 8 and 20
  public static final Pose ReefPosition3 =
      new Pose(
          new Pose2d(5.169, 5.195, Rotation2d.fromDegrees(-120)),
          new Pose2d(12.391, 2.814, Rotation2d.fromDegrees(60)));

  // tag numbers 7 and 21
  public static final Pose ReefPosition4 =
      new Pose(
          new Pose2d(5.835, 4.023, Rotation2d.fromDegrees(180)),
          new Pose2d(11.697, 3.977, Rotation2d.fromDegrees(0)));

  // tag numbers 6 and 22
  public static final Pose ReefPosition5 =
      new Pose(
          new Pose2d(5.169, 2.865, Rotation2d.fromDegrees(120)),
          new Pose2d(12.391, 5.192, Rotation2d.fromDegrees(-60)));

  // tag numbers 11 and 17
  public static final Pose ReefPosition6 =
      new Pose(
          new Pose2d(3.839, 2.836, Rotation2d.fromDegrees(60)),
          new Pose2d(13.745, 5.209, Rotation2d.fromDegrees(-120)));
}
