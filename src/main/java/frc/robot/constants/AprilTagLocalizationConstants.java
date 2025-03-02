// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class AprilTagLocalizationConstants {
  public static class LimelightDetails {
    public String name;
    public Matrix<N3, N1> closeStdDevs;
    public Matrix<N3, N1> farStdDevs;
    public Matrix<N3, N1> inverseOffset;

    /*
     *
     */
    public LimelightDetails(
        String name,
        Matrix<N3, N1> closeStdDevs,
        Matrix<N3, N1> farStdDevs,
        Matrix<N3, N1> inverseOffset) {
      this.name = name;
      this.closeStdDevs = closeStdDevs;
      this.farStdDevs = farStdDevs;
      this.inverseOffset = inverseOffset;
    }
  }

  public static final String LIMELIGHT_NAME_BACKL = "limelight-backl";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_BACKL =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_BACKL =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_INVERSE_OFFSET_BACKL =
      VecBuilder.fill(0.294, -0.303, -0.199);
  public static final LimelightDetails LIMELIGHT_DETAILS_BACKL =
      new LimelightDetails(
          LIMELIGHT_NAME_BACKL,
          LIMELIGHT_CLOSE_STDDEV_BACKL,
          LIMELIGHT_FAR_STDDEV_BACKL,
          LIMELIGHT_INVERSE_OFFSET_BACKL);

  public static final String LIMELIGHT_NAME_ELEVATE = "limelight-elevate";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_ELEVATE =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_ELEVATE =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_INVERSE_OFFSET_ELEVATE =
      VecBuilder.fill(-0.17, -0.251, -0.949);
  public static final LimelightDetails LIMELIGHT_DETAILS_ELEVATE =
      new LimelightDetails(
          LIMELIGHT_NAME_ELEVATE,
          LIMELIGHT_CLOSE_STDDEV_ELEVATE,
          LIMELIGHT_FAR_STDDEV_ELEVATE,
          LIMELIGHT_INVERSE_OFFSET_ELEVATE);
  public static final String LIMELIGHT_NAME_FRONTR = "limelight-frontr";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_FRONTR =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_FRONTR =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_INVERSE_OFFSET_FRONTR =
      VecBuilder.fill(-0.31, 0.24, -0.229);
  public static final LimelightDetails LIMELIGHT_DETAILS_FRONTR =
      new LimelightDetails(
          LIMELIGHT_NAME_FRONTR,
          LIMELIGHT_CLOSE_STDDEV_FRONTR,
          LIMELIGHT_FAR_STDDEV_FRONTR,
          LIMELIGHT_INVERSE_OFFSET_FRONTR);

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final Distance MAX_TAG_DISTANCE = Meters.of(3.0);
  public static final Time LOCALIZATION_PERIOD = Seconds.of(0.02);
}
