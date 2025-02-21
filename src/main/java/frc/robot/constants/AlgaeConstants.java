// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class AlgaeConstants {
  public static final AngularVelocity INTAKE_SPEED = RevolutionsPerSecond.of(3);
  public static final AngularVelocity OUTTAKE_SPEED = RevolutionsPerSecond.of(-3);
  public static final AngularVelocity SHOOTING_SPEED = RevolutionsPerSecond.of(-10);
}
