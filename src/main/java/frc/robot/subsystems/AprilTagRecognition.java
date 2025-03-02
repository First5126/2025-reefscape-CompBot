package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.ApriltagConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ApriltagConstants.Blue;
import frc.robot.constants.ApriltagConstants.Red;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.HashMap;

public class AprilTagRecognition extends SubsystemBase {
  private HashMap<Integer, Command> m_AprilTagHashMap;
  private CommandFactory m_commandFactory;
  private int m_currentAprilID;
  private boolean initialized = false;

  public AprilTagRecognition(CommandFactory commandFactory) {
    m_commandFactory = commandFactory;
  }

  private int getClosestTagId() {
    RawFiducial[] allTags =
        LimelightHelpers.getRawFiducials(AprilTagLocalizationConstants.LIMELIGHT_NAME_FRONTR);
    RawFiducial closestTag;
    int result = 0;
    SmartDashboard.putNumber("Tag length", allTags.length);
    if (allTags.length > 0) {
      closestTag = allTags[0];
      for (RawFiducial tag : allTags) {
        if (tag.distToRobot < closestTag.distToRobot) {
          closestTag = tag;
        }
      }
      result = closestTag.id;
    } else {
      result = 0;
    }
    SmartDashboard.putNumber("Current April Tag Command ID", result);

    return result;
  }

  public Command getAprilTagCommand() {
    return Commands.deferredProxy(
        () -> {
            final int tagId = getClosestTagId();
            Blue blueOption = Blue.fromId(tagId);
            Red redOption = Red.fromId(tagId);

            if (blueOption != null) {
                switch (blueOption) {
                    case LEFT_CORAL_STATION:
                        return m_commandFactory.goToPose(PoseConstants.leftCoralStationPosition2.getPose());
                    case RIGHT_CORAL_STATION:
                        return m_commandFactory.goToPose(PoseConstants.rightCoralStationPosition2.getPose());
                    case LEFT_BARGE:
                        return m_commandFactory.goToPose(PoseConstants.LeftBarge.getPose());
                    case RIGHT_BARGE:
                        return m_commandFactory.goToPose(PoseConstants.rightBarge.getPose());
                    case PROCESSOR:
                        return m_commandFactory.goToPose(PoseConstants.prossesor.getPose());
                    case REEF_6:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition6.getPose());
                    case REEF_1:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition1.getPose());
                    case REEF_2:
                        Command cmd = m_commandFactory.goToPose(PoseConstants.ReefPosition2.getPose());
                        if (cmd == null) {
                            System.out.println("NO COMMAND FOUND");
                        } else {
                            System.out.println("COMMAND FOUND");
                        }
                        return cmd;
                    case REEF_3:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition3.getPose());
                    case REEF_4:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition4.getPose());
                    case REEF_5:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition5.getPose());

                    default:
                        System.out.println("I did not find a value blue");
                        break;
                }
            } else if (redOption != null) {
                switch (redOption) {
                    case LEFT_CORAL_STATION:
                        return m_commandFactory.goToPose(PoseConstants.leftCoralStationPosition2.getPose());
                    case RIGHT_CORAL_STATION:
                        return m_commandFactory.goToPose(PoseConstants.rightCoralStationPosition2.getPose());
                    case PROCESSOR:
                        return m_commandFactory.goToPose(PoseConstants.prossesor.getPose());
                    case LEFT_BARGE:
                        return m_commandFactory.goToPose(PoseConstants.LeftBarge.getPose());
                    case RIGHT_BARGE:
                        return m_commandFactory.goToPose(PoseConstants.rightBarge.getPose());
                    case REEF_2:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition2.getPose());
                    case REEF_1:
                        Command cmd = m_commandFactory.goToPose(PoseConstants.ReefPosition1.getPose());
                        if (cmd == null) {
                            System.out.println("COMMAND NOT FOUND RED");
                        } else {
                            System.out.println("COMMAND FOUND RED");
                        }
                        return cmd;
                    case REEF_6:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition6.getPose());
                    case REEF_5:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition5.getPose());
                    case REEF_4:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition4.getPose());
                    case REEF_3:
                        return m_commandFactory.goToPose(PoseConstants.ReefPosition3.getPose());

                    default:
                        System.out.println("I did not find a value Red");
                        break;
                }
            }
            return Commands.none();
        });
  }

  public void periodic() {

  }
}
