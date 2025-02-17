package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseConstants {

  public static class Pose {
    private Pose2d redPose,bluePose;
    public Pose(Pose2d bluePose, Pose2d redPose) {
      this.bluePose = bluePose;
      this.redPose = redPose;
    }
    public Pose2d getPose(){
      if (DriverStation.getAlliance().get().name().equals("red")){
        return redPose;
      } else {
        return bluePose;
      } 
    }
  }
  
  //TODO: None of these positions are true. They were all in testing and will NOT WORK AT ALL.

  //tag numbers 1 and 13
  public static final Pose leftCoralStationPosition1 = new Pose(
    new Pose2d(1.7,7.3,Rotation2d.fromDegrees(125)),
    new Pose2d(16.8,1.4,Rotation2d.fromDegrees(-55))
    );
  public static final Pose leftCoralStationPosition2 = new Pose(
    new Pose2d(1.3,7,Rotation2d.fromDegrees(125)),
    new Pose2d(16.2,1,Rotation2d.fromDegrees(-55))
    );
  public static final Pose leftCoralStationPosition3 = new Pose(
    new Pose2d(0.8,6.6,Rotation2d.fromDegrees(125)),
    new Pose2d(15.7,0.6,Rotation2d.fromDegrees(-55))
    );

    //tag numbers 2 and 12
    public static final Pose rightCoralStationPosition1 = new Pose(
    new Pose2d(1.7,7.3,Rotation2d.fromDegrees(125)),
    new Pose2d(16.8,1.4,Rotation2d.fromDegrees(-55))
    );
  public static final Pose rightCoralStationPosition2 = new Pose(
    new Pose2d(1.3,7,Rotation2d.fromDegrees(125)),
    new Pose2d(16.2,1,Rotation2d.fromDegrees(-55))
    );
  public static final Pose rightCoralStationPosition3 = new Pose(
    new Pose2d(0.8,6.6,Rotation2d.fromDegrees(125)),
    new Pose2d(15.7,0.6,Rotation2d.fromDegrees(-55))
    );

  //tag numbers 3 and 16
  public static final Pose prossesor = new Pose(
    new Pose2d(6.4,0.6,Rotation2d.fromDegrees(-90)),
    new Pose2d(11.5,7.45,Rotation2d.fromDegrees(90))
    );

  //tag numbers 4 and 15
  public static final Pose rightBarge = new Pose(
    new Pose2d(6.4,0.6,Rotation2d.fromDegrees(-90)),
    new Pose2d(11.5,7.45,Rotation2d.fromDegrees(90))
    );

    //tag numbers 5 and 14
  public static final Pose LeftBarge = new Pose(
    new Pose2d(6.4,0.6,Rotation2d.fromDegrees(-90)),
    new Pose2d(11.5,7.45,Rotation2d.fromDegrees(90))
    );

    //tag numbers 10 and 18
    public static final Pose ReefPosition1 = new Pose(
    new Pose2d(0.95,4,Rotation2d.fromDegrees(0)),
    new Pose2d(11.66,4.01,Rotation2d.fromDegrees(0))
    ); 

    //tag numbers 9 and 19
    public static final Pose ReefPosition2 = new Pose(
    new Pose2d(3.8,5.2,Rotation2d.fromDegrees(-55)),
    new Pose2d(12.3,5.2,Rotation2d.fromDegrees(-55))
    ); 

    //tag numbers 8 and 20
    public static final Pose ReefPosition3 = new Pose(
    new Pose2d(5.184,5.235,Rotation2d.fromDegrees(-120)),
    new Pose2d(13.75,5.22,Rotation2d.fromDegrees(-120))
    ); 

    //tag numbers 7 and 21
    public static final Pose ReefPosition4 = new Pose(
    new Pose2d(5.9,4,Rotation2d.fromDegrees(180)),
    new Pose2d(14.5,4,Rotation2d.fromDegrees(180))
    ); 

    //tag numbers 6 and 22
    public static final Pose ReefPosition5 = new Pose(
    new Pose2d(5.2,2.8,Rotation2d.fromDegrees(120)),
    new Pose2d(13.7,2.7,Rotation2d.fromDegrees(120))
    ); 

    //tag numbers 11 and 17
    public static final Pose ReefPosition6 = new Pose(
    new Pose2d(3.74,2.8,Rotation2d.fromDegrees(55)),
    new Pose2d(12.36,2.85,Rotation2d.fromDegrees(55))
    ); 

}