// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class CoralConstants {
  public static final AngularVelocity INTAKE_SPEED = RevolutionsPerSecond.of(60);
  public static final AngularVelocity OUTTAKE_SPEED = RevolutionsPerSecond.of(-110);

  public static final double PROXIMITY_THRESHOLD = 0.1;
  public static final double DEBOUNCE = 0.00;

  public static final double kP = 20;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;
}
