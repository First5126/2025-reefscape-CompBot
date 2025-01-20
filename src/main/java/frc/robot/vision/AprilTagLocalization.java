// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.vision.AprilTagLocalizationConstants.MAX_TAG_DISTANCE;
import static frc.robot.vision.AprilTagLocalizationConstants.LOCALIZATION_PERIOD;
import frc.robot.vision.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.vision.LimelightHelpers.PoseEstimate;



/**
 * A class that uses the limelight to localize the robot using AprilTags.
 * it runs in a background thread instead of the main robot loop.
 */
public class AprilTagLocalization {
  private Notifier m_notifier = new Notifier(this::periodic);  //calls pose estimate on the the period
  private LimelightDetails[] m_LimelightDetails;  // list of limelights that can provide updates
  private Supplier<Pose2d> m_robotPoseSupplier;  // supplies the pose of the robot
  public boolean m_fullTrust;  //to allow for button trust the tag estimate over all else.
  public double oldyawDegrees = 0;
  private MutAngle m_yaw = Degrees.mutable(0); ;
  public MutAngle M_OldYaw = Degrees.mutable(0);  
  private VisionConsumer m_VisionConsumer;
 
  
  private double maxFieldDistanceX = 0.0;
  private double maxFieldDistanceY = 0.0;


  /**
   * Creates a new AprilTagLocalization.
   * @param poseSupplier supplies the current robot pose
   * @param visionConsumer // a consumer that accepts the vision pose, timestamp, and std deviations
   * @param details // the details of the limelight, more than one can be passed to allow for multipe on the bot.
   */
  public AprilTagLocalization(Supplier<Pose2d> poseSupplier, VisionConsumer visionConsumer,  LimelightDetails ... details) {
    m_notifier.startPeriodic(0.02); // set up a pose estimation loop with a 0.02 second period.
    m_LimelightDetails = details;
    m_robotPoseSupplier = poseSupplier;
  }

  /**
   * Sets the full trust of the vision system.  The robot will trust the vision system over all other sensors.
   */
  public void setFullTrust(boolean fullTrust) {
    this.m_fullTrust = fullTrust;
  }
  private boolean isPoseOnfield(Pose2d observation) { 
    // Coordinates for where Pose is on the field
    if (
      observation.getX() < 0.0 // What X position robot is on the field.
      || observation.getY() < 0.0 // What Y position robot is on the field.
      || observation.getX() > maxFieldDistanceX // Whether the robot X position is on the field or not
      || observation.getY() > maxFieldDistanceY // Whether the robot X position is on the field or not
    ){
      return true;
    } else {
      return false;
    }
  }
  /**
   * Interpolates between two std deviations.
   * @param closeStdDevs
   * @param farStdDevs
   * @param scale
   * @return
   */
  public Matrix<N3, N1> interpolate(Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale){
    return VecBuilder.fill(MathUtil.interpolate(closeStdDevs.get(0,0), farStdDevs.get(0,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(1,0), farStdDevs.get(1,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(2,0), farStdDevs.get(2,0), scale));
  }



   /**
   * Estimates the pose of the robot using the limelight.
   * This function will run in a background thread once per AprilTagLocalizationConstants.LOCALIZATION_PERIOD.
   */
  public void periodic() {
    for (LimelightDetails limelightDetail : m_LimelightDetails) {
      m_yaw.mut_replace(Degrees.of(m_robotPoseSupplier.get().getRotation().getDegrees()));
      // Yaw Rate is calculated by the difference between the current yaw and the old yaw divided by the localization period
      AngularVelocity yawRate = (m_yaw.minus(M_OldYaw).div(LOCALIZATION_PERIOD));
      // Orientation is set from LimelightHelpers
      LimelightHelpers.SetRobotOrientation(limelightDetail.name, m_yaw.in(Degrees), yawRate.in(DegreesPerSecond),0,0,0,0 );  // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      // Pose Estimate from LimelightHelpers
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightDetail.name);  // Retrieving the current pose estimation from the Limelight using the specified Limelight detail
      if (poseEstimate != null) {
        // If the pose estimate is not null, then the pose is printed to the console
      System.out.println("Pose X: " + poseEstimate.pose.getX());
      System.out.println("Pose Y: " + poseEstimate.pose.getY());
      SmartDashboard.putNumber("Pose Estimate X", poseEstimate.pose.getX());
      SmartDashboard.putNumber("Pose Estimate Y", poseEstimate.pose.getY());
      double scale = poseEstimate.avgTagDist / MAX_TAG_DISTANCE.in(Meters); // Dividing the average tag distance by the max tag distance
      // If Else statement to determine if the pose is on the field and if the pose is within the max tag distance
      if (m_fullTrust) { // Full Trust is when the robot trusts the vision system over all other sensors
        m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, limelightDetail.closeStdDevs);
      } else if (isPoseOnfield(poseEstimate.pose) && poseEstimate.avgTagDist < MAX_TAG_DISTANCE.in(Meters)) { // rejects poses that are more than max tag distance we trust
        // If the Pose on the field is less than the max tag distance, then the poses are rejected
        Matrix<N3,N1> interpolated = interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
        // Pose is set to the vision consumer
        m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
      }
      M_OldYaw.mut_replace(m_yaw);
    }
  }
}
  
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }
}