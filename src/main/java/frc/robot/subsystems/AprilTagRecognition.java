package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.ApriltagConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ClimbingConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class AprilTagRecognition extends SubsystemBase {
  private HashMap<Integer, Command> m_AprilTagHashMap;
  private CommandFactory m_commandFactory;
  private int m_currentAprilID = 0;

  public AprilTagRecognition(CommandFactory commandFactory) {
    m_commandFactory = commandFactory;
    m_AprilTagHashMap = new HashMap<>(23);
    m_AprilTagHashMap.put(0, Commands.none());
    m_AprilTagHashMap.put(ApriltagConstants.Red.LEFT_CORAL_STATION.id, m_commandFactory.goToPose(PoseConstants.leftCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.RIGHT_CORAL_STATION.id, m_commandFactory.goToPose(PoseConstants.rightCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.PROCESSOR.id, m_commandFactory.goToPose(PoseConstants.prossesor.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.LEFT_BARGE.id, m_commandFactory.goToPose(PoseConstants.LeftBarge.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.RIGHT_BARGE.id, m_commandFactory.goToPose(PoseConstants.rightBarge.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_1.id, m_commandFactory.goToPose(PoseConstants.ReefPosition1.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_2.id, m_commandFactory.goToPose(PoseConstants.ReefPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_3.id, m_commandFactory.goToPose(PoseConstants.ReefPosition3.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_4.id, m_commandFactory.goToPose(PoseConstants.ReefPosition4.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_5.id, m_commandFactory.goToPose(PoseConstants.ReefPosition5.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Red.REEF_6.id, m_commandFactory.goToPose(PoseConstants.ReefPosition6.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.LEFT_CORAL_STATION.id, m_commandFactory.goToPose(PoseConstants.leftCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.RIGHT_CORAL_STATION.id, m_commandFactory.goToPose(PoseConstants.rightCoralStationPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.PROCESSOR.id, m_commandFactory.goToPose(PoseConstants.prossesor.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.LEFT_BARGE.id, m_commandFactory.goToPose(PoseConstants.LeftBarge.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.RIGHT_BARGE.id, m_commandFactory.goToPose(PoseConstants.rightBarge.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_1.id, m_commandFactory.goToPose(PoseConstants.ReefPosition1.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_2.id, m_commandFactory.goToPose(PoseConstants.ReefPosition2.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_3.id, m_commandFactory.goToPose(PoseConstants.ReefPosition3.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_4.id, m_commandFactory.goToPose(PoseConstants.ReefPosition4.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_5.id, m_commandFactory.goToPose(PoseConstants.ReefPosition5.getPose()));
    m_AprilTagHashMap.put(ApriltagConstants.Blue.REEF_6.id, m_commandFactory.goToPose(PoseConstants.ReefPosition6.getPose()));
  }


  private int getClosestTagId(){
    RawFiducial[] allTags = LimelightHelpers.getRawFiducials(AprilTagLocalizationConstants.LIMELIGHT_NAME);
    RawFiducial closestTag;
    int result = 0;
    if (allTags.length > 0) {
      closestTag = allTags[0];
      result = closestTag.id;
    } else {
      result = 0;
    }
    
    return result;
  }


  public Command getAprilTagCommand() {
    //TODO: make sure this works
    Command tagCommand = Commands.select(m_AprilTagHashMap, this::getClosestTagId);
    m_currentAprilID = this.getClosestTagId();
    return tagCommand;
  }


  public void periodic() {
    SmartDashboard.putNumber("Current April Tag Command ID", m_currentAprilID);
  }
}
