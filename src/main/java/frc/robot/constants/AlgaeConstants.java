// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeConstants {
  public static final Voltage INTAKE_SPEED = Volts.of(6);
  public static final Voltage OUTTAKE_SPEED = Volts.of(-8);
  public static final Voltage SHOOTING_SPEED = Volts.of(1);
  public static final Voltage HOLDING_SPEED = Volts.of(3);

  public static final double kP = 5;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final double PROXIMITY_THRESHOLD = 0.1;
  public static final double DEBOUNCE = 0.02;
}
