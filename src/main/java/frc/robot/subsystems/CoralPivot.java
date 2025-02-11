// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
  private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  private TalonFXConfiguration m_talonConfiguration;
  private TalonFX m_CoralPivotTalon;

  public CoralPivot() {    
    m_talonConfiguration = new TalonFXConfiguration();
    m_talonConfiguration.CurrentLimits.SupplyCurrentLimit = CoralPivotConstants.supplyCurrentLimit;
    m_talonConfiguration.CurrentLimits.SupplyCurrentLowerLimit = CoralPivotConstants.lowerSupplyCurrentLimit;
    m_talonConfiguration.Slot0.kP = CoralPivotConstants.kP;
    m_talonConfiguration.Slot0.kI = CoralPivotConstants.kI;
    m_talonConfiguration.Slot0.kD = CoralPivotConstants.kD;
    m_talonConfiguration.Slot0.kG = CoralPivotConstants.kG;
    m_talonConfiguration.Slot0.kA = CoralPivotConstants.kA;
    m_talonConfiguration.Slot0.kV = CoralPivotConstants.kV;
    
    m_CoralPivotTalon = new TalonFX(CoralPivotConstants.MotorID);
    m_CoralPivotTalon.getConfigurator().apply(m_talonConfiguration);
    m_CoralPivotTalon.setNeutralMode(NeutralModeValue.Brake);
  }

  private void rotateArm(Angle setpoint) {
    m_CoralPivotTalon.setControl(positionVoltage.withPosition(setpoint));
  }

  public Command setArmSetpoint(Angle position) {
    return runOnce(
      () -> {
        rotateArm(position);
      });
  }
}
