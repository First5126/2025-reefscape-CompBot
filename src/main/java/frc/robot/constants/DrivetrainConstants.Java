// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DrivetrainConstants {
  public static final double maxSpeedMetersPerSecond = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double trackWidthMeters = Units.inchesToMeters(18.78);
  public static final double rotationDiameter = trackWidthMeters * Math.PI * Math.sqrt(2);
  public static final double rotationsPerSecond = maxSpeedMetersPerSecond / rotationDiameter;
  public static final double maxAngularVelocityRadiansPerSecond = 2 * Math.PI * rotationsPerSecond;
  public static Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  public static final double currentLimit = 60;
  public static final double autoMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond * 0.8;
  public static final double estimatedKp = 12/(maxSpeedMetersPerSecond/ (TunerConstants.kWheelRadius.in(Meters) * 2 * Math.PI));
  public static final PathConstraints pathConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  public class CurrentLimits {
    public static final Current kDriveCurrentLimitMax = Amps.of(70); //Max draw allowed
    public static final Current  kDriveCurrentLimitMin = Amps.of(40); //Motor drops to min after hitting ma
    public static final Time kDriveCurrentDuration = Seconds.of(0.25); //Time to hold at min current
  }
}