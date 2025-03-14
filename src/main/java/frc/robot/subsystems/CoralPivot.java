// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.CoralPivotConstants;

public class CoralPivot extends SubsystemBase {
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private TalonFXSConfiguration m_talonConfiguration;
  private TalonFXS m_CoralPivotTalon;
  private CoreCANcoder m_CANcoder;

  public CoralPivot() {
    CANcoderConfiguration CANCoderConfiguration = new CANcoderConfiguration();
    CANCoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    CANCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    m_CANcoder = new CoreCANcoder(CANConstants.CORAL_PIVOT_CANCODER);
    m_CANcoder.getConfigurator().apply(CANCoderConfiguration);

    m_talonConfiguration = new TalonFXSConfiguration();
    m_talonConfiguration.CurrentLimits.SupplyCurrentLimit = CoralPivotConstants.supplyCurrentLimit;
    m_talonConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
        CoralPivotConstants.lowerSupplyCurrentLimit;
    m_talonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_talonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_talonConfiguration.ExternalFeedback.RotorToSensorRatio = 75.0;
    m_talonConfiguration.ExternalFeedback.SensorToMechanismRatio = 1.0;

    m_talonConfiguration.ExternalFeedback.withFusedCANcoder(m_CANcoder);

    m_talonConfiguration.MotionMagic.MotionMagicAcceleration =
        CoralPivotConstants.MotionMagicAcceleration;
    m_talonConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        CoralPivotConstants.MotionMagicCruiseVelocity;
    m_talonConfiguration.MotionMagic.MotionMagicJerk = CoralPivotConstants.MotionMagicJerk;

    m_talonConfiguration.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

    m_talonConfiguration.Slot0.kP = CoralPivotConstants.kP;
    m_talonConfiguration.Slot0.kI = CoralPivotConstants.kI;
    m_talonConfiguration.Slot0.kD = CoralPivotConstants.kD;
    m_talonConfiguration.Slot0.kG = CoralPivotConstants.kG;
    m_talonConfiguration.Slot0.kA = CoralPivotConstants.kA;
    m_talonConfiguration.Slot0.kV = CoralPivotConstants.kV;

    m_CoralPivotTalon = new TalonFXS(CANConstants.CORAL_PIVOT, CANConstants.ELEVATOR_CANIVORE);
    m_CoralPivotTalon.getConfigurator().apply(m_talonConfiguration);
    m_CoralPivotTalon.setNeutralMode(NeutralModeValue.Brake);
  }

  private void rotate(Angle setpoint) {
    m_CoralPivotTalon.setControl(motionMagicVoltage.withPosition(setpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Coral Pivot Degrees", m_CoralPivotTalon.getPosition().getValue().in(Degrees));

    SmartDashboard.putNumber(
        "Coral Pivot Motor Temperature",
        m_CoralPivotTalon.getExternalMotorTemp().getValueAsDouble());
  }

  public Command goToLowerSetpoint() {
    return runOnce(
        () -> {
          rotate(CoralPivotConstants.LOWER_ANGLE);
        });
  }

  public Command goToUpperSetpoint() {
    return runOnce(
        () -> {
          rotate(CoralPivotConstants.UPPER_ANGLE);
        });
  }

  public Command gotoCoralStationSetpoint() {
    return runOnce(
        () -> {
          rotate(CoralPivotConstants.CORAL_STATION_ANGLE);
        });
  }

  public Command gotoAngle(Angle angle) {
    return runOnce(
        () -> {
          rotate(angle);
        });
  }

  public void disable() {
    m_CoralPivotTalon.setControl(new DutyCycleOut(0));
  }
}
