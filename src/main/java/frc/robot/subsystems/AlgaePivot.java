// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.AlgorithmConstraints;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaePivotConstants;
import frc.robot.constants.CANConstants;

public class AlgaePivot extends SubsystemBase {
    private Slot0Configs m_Slot0Configs;  
    private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private TalonFX m_AlgaePivotTalon;
    private TalonFXConfiguration m_TalonConfiguration;

    public AlgaePivot() {
        m_Slot0Configs = new Slot0Configs();
        m_Slot0Configs.kP = AlgaePivotConstants.kP;
        m_Slot0Configs.kI = AlgaePivotConstants.kI;
        m_Slot0Configs.kD = AlgaePivotConstants.kD;
        m_Slot0Configs.kG = AlgaePivotConstants.kG;
        m_Slot0Configs.kV = AlgaePivotConstants.kV;
        m_Slot0Configs.kA = AlgaePivotConstants.kA;
        
        m_TalonConfiguration = new TalonFXConfiguration();
        m_TalonConfiguration.CurrentLimits.SupplyCurrentLimit = AlgaePivotConstants.supplyCurrentLimit;
        m_TalonConfiguration.CurrentLimits.SupplyCurrentLowerLimit = AlgaePivotConstants.lowerSupplyCurrentLimit;

        m_AlgaePivotTalon = new TalonFX(CANConstants.ALGAE_PIVOT);
        m_AlgaePivotTalon.setNeutralMode(NeutralModeValue.Brake);
        m_AlgaePivotTalon.getConfigurator().apply(m_Slot0Configs);
        m_AlgaePivotTalon.getConfigurator().apply(m_TalonConfiguration);
    }

    private void rotate(Angle setpoint) {
        m_AlgaePivotTalon.setControl(positionVoltage.withPosition(setpoint));
      }
    
    public Command goToLowerSetpoint() {
      return runOnce(
        () -> {
          rotate(AlgaePivotConstants.LOWER_ANGLE);
        });
    }
    
    public Command goToUpperSetpoint() {
      return runOnce(
        () -> {
          rotate(AlgaePivotConstants.UPPER_ANGLE);
        });
    }
}
