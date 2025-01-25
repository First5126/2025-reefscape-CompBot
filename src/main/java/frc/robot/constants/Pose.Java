package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Pose { 
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