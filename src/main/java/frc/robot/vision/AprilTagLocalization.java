// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.vision.AprilTagLocalizationConstants.LimelightDetails;
import static frc.robot.vision.AprilTagLocalizationConstants.FIELD_LAYOUT;
import static frc.robot.vision.AprilTagLocalizationConstants.MAX_TAG_DISTANCE;
import static frc.robot.vision.AprilTagLocalizationConstants.LOCALIZATION_PERIOD;
import frc.robot.vision.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.vision.LimelightHelpers.PoseEstimate;


/**
 * A class that uses the limelight to localize the robot using AprilTags.
 * it runs in a background thread instead of the main robot loop.
 */
public class AprilTagLocalization extends SubsystemBase{
  private LimelightDetails[] m_LimelightDetails;  // list of limelights that can provide updates
  private Supplier<Pose2d> m_robotPoseSupplier;  // supplies the pose of the robot
  private boolean m_FullTrust;  //to allow for button trust the tag estimate over all else.
  private MutAngle m_yaw = Degrees.mutable(0); ;
  private MutAngle m_OldYaw = Degrees.mutable(0);  // the previous yaw 
  private VisionConsumer m_VisionConsumer;
  private PoseReset m_poseReset;


  /**
   * Creates a new AprilTagLocalization.
   * @param poseSupplier supplies the current robot pose
   * @param visionConsumer // a consumer that accepts the vision pose, timestamp, and std deviations
   */
  public AprilTagLocalization(Supplier<Pose2d> poseSupplier, VisionConsumer visionConsumer) {
    LimelightDetails[] details = new LimelightDetails[0];
        m_LimelightDetails = details;
    m_robotPoseSupplier = poseSupplier;
    m_VisionConsumer = visionConsumer; 
    PoseReset poseReset = (pose) -> {}; // Initialize with a default implementation
    m_poseReset = poseReset;
  }

  /**
   * Sets the full trust of the vision system.  The robot will trust the vision system over all other sensors.
   * @param fullTrust
   */
  public void setFullTrust(boolean fullTrust) {
    m_FullTrust = fullTrust;
  }

  public Command setTrust(boolean fullTrust) {
    return runOnce(
      () -> {
        m_FullTrust = fullTrust;
      });
  }


  /**
   * Checks if the pose is on the field.
   * @param observation
   * @return
   */
  private boolean isPoseOnfield(Pose2d observation) { 
    // Coordinates for where Pose is on the field
    return observation.getX() < 0.0 // What X position robot is on the field.
        || observation.getY() < 0.0 // What Y position robot is on the field.
        || observation.getX() > FIELD_LAYOUT.getFieldLength()// Whether the robot X position is on the field or not
        || observation.getY() > FIELD_LAYOUT.getFieldWidth(); // Whether the robot X position is on the field or not
  }

  /**
   * Interpolates between two std deviations.
   * @param closeStdDevs
   * @param farStdDevs
   * @param scale
   * @return
   */
  private Matrix<N3, N1> interpolate(Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale){
    return VecBuilder.fill(MathUtil.interpolate(closeStdDevs.get(0,0), farStdDevs.get(0,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(1,0), farStdDevs.get(1,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(2,0), farStdDevs.get(2,0), scale));
  }

  /**
   * Estimates the pose of the robot using the limelight.
   * This function will run in a background thread once per AprilTagLocalizationConstants.LOCALIZATION_PERIOD.
   */
  @Override
  public void periodic() {
    for (LimelightDetails limelightDetail : m_LimelightDetails) {
      m_yaw.mut_replace(Degrees.of(m_robotPoseSupplier.get().getRotation().getDegrees()));
      AngularVelocity yawRate = (m_yaw.minus(m_OldYaw).div(LOCALIZATION_PERIOD));
      // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      LimelightHelpers.SetRobotOrientation(limelightDetail.name, m_yaw.in(Degrees), yawRate.in(DegreesPerSecond),0,0,0,0 );  // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      // Get the pose from the Limelight
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightDetail.name);  // Get the pose from the Limelight 
      if (poseEstimate != null) {
        System.out.println("Pose X: " + poseEstimate.pose.getX());
        System.out.println("Pose Y: " + poseEstimate.pose.getY());
        double scale = poseEstimate.avgTagDist / MAX_TAG_DISTANCE.in(Meters); 
        if (m_FullTrust) {
          m_poseReset.accept(poseEstimate.pose);
        } else if (isPoseOnfield(poseEstimate.pose) && poseEstimate.avgTagDist < MAX_TAG_DISTANCE.in(Meters)) { // reject poses that are more than max tag distance we trust
          Matrix<N3,N1> interpolated = interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
          m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
          System.out.println("Pose X: " + poseEstimate.pose.getX());
          System.out.println("Pose Y: " + poseEstimate.pose.getY());
        }
        m_OldYaw.mut_replace(m_yaw);
      } else{
        System.out.println("No readings");
      }
    }
  }
  
  /** Defines a function pointer to a function with a signature
   * ( Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs)
   * to accept the vision pose estimate
   * */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public interface PoseReset {
    void accept(
      Pose2d visionRobotPoseMeters);
  }
}