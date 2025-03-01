package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.ApriltagConstants.Blue;
import frc.robot.constants.ApriltagConstants.Red;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import java.util.HashMap;

public class AprilTagRecognition extends SubsystemBase {
  private HashMap<Integer, Command> m_AprilTagHashMap;
  private CommandFactory m_commandFactory;
  private int m_currentAprilID = 0;

  public AprilTagRecognition(CommandFactory commandFactory) {
    m_commandFactory = commandFactory;
  }

  private int getClosestTagId() {
    RawFiducial[] allTags =
        LimelightHelpers.getRawFiducials(AprilTagLocalizationConstants.LIMELIGHT_NAME);
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
    // TODO: make sure this works
    Command tagCommand = Commands.select(m_AprilTagHashMap, this::getClosestTagId);
    m_currentAprilID = this.getClosestTagId();
    return tagCommand;
  }

  public void periodic() {
    SmartDashboard.putNumber("Current April Tag Command ID", m_currentAprilID);
  }

  public Command getPosition() {
    return Commands.deferredProxy(
        () -> {
          m_currentAprilID = getClosestTagId();

          Blue blueOption = Blue.fromId(m_currentAprilID);
          Red redOption = Red.fromId(m_currentAprilID);
          if (blueOption != null) {
            return m_commandFactory.goToPose(blueOption.pose[0].getPose());
          } else if (redOption != null) {
            return m_commandFactory.goToPose(redOption.pose[0].getPose());
          } else {
            System.out.println("Command was not found for position");
          }
          return Commands.none();
        });
  }
}
