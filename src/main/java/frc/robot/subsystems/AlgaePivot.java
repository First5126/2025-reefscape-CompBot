// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaePivotConstants;
import frc.robot.constants.CANConstants;

public class AlgaePivot extends SubsystemBase {
  private Slot0Configs m_Slot0Configs;
  private MotionMagicVoltage m_motionMatiVoltage = new MotionMagicVoltage(0).withSlot(0);
  private TalonFXS m_AlgaePivotTalon;
  private TalonFXSConfiguration m_TalonConfiguration;

  public AlgaePivot() {
    m_Slot0Configs = new Slot0Configs();
    m_Slot0Configs.kP = AlgaePivotConstants.kP;
    m_Slot0Configs.kI = AlgaePivotConstants.kI;
    m_Slot0Configs.kD = AlgaePivotConstants.kD;
    m_Slot0Configs.kG = AlgaePivotConstants.kG;
    m_Slot0Configs.kV = AlgaePivotConstants.kV;
    m_Slot0Configs.kA = AlgaePivotConstants.kA;

    m_TalonConfiguration = new TalonFXSConfiguration();
    m_TalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_TalonConfiguration.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
    m_TalonConfiguration.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.PulseWidth;
    m_TalonConfiguration.ExternalFeedback.QuadratureEdgesPerRotation = 2048;
    m_TalonConfiguration.ExternalFeedback.RotorToSensorRatio = 100;
    m_TalonConfiguration.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_TalonConfiguration.CurrentLimits.SupplyCurrentLimit = AlgaePivotConstants.supplyCurrentLimit;
    m_TalonConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
        AlgaePivotConstants.lowerSupplyCurrentLimit;

    m_AlgaePivotTalon = new TalonFXS(CANConstants.ALGAE_PIVOT, CANConstants.ELEVATOR_CANIVORE);
    m_AlgaePivotTalon.setNeutralMode(NeutralModeValue.Brake);
    m_AlgaePivotTalon.getConfigurator().apply(m_Slot0Configs);
    m_AlgaePivotTalon.getConfigurator().apply(m_TalonConfiguration);

    m_AlgaePivotTalon.setControl(new DutyCycleOut(0));
  }

  private void rotate(Angle setpoint) {
    m_AlgaePivotTalon.setControl(m_motionMatiVoltage.withPosition(setpoint));
  }

  public Command goToLowerSetpoint() {
    return runOnce(
        () -> {
          rotate(AlgaePivotConstants.LOWER_ANGLE);
        });
  }

  public Command goToProssesorSetpoint() {
    return runOnce(
        () -> {
          rotate(AlgaePivotConstants.PROSSESSOR_ANGLE);
        });
  }

  public Command goToUpperSetpoint() {
    return runOnce(
        () -> {
          rotate(AlgaePivotConstants.UPPER_ANGLE);
        });
  }
}
