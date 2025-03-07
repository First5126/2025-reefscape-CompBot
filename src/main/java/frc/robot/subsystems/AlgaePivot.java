// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlgaePivotConstants;
import frc.robot.constants.CANConstants;
import frc.robot.constants.CoralLevels;
import java.util.function.Supplier;

public class AlgaePivot extends SubsystemBase {
  private MotionMagicVoltage m_motionMatiVoltage = new MotionMagicVoltage(0).withSlot(0);
  private TalonFXS m_AlgaePivotTalon;
  private Trigger hasAlgae;
  private Supplier<CoralLevels> coralLevelSupplier;

  public AlgaePivot(Trigger hasAlgae, Supplier<CoralLevels> coralLevelSupplier) {
    this.hasAlgae = hasAlgae;
    this.coralLevelSupplier = coralLevelSupplier;

    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();

    talonFXSConfiguration.Slot0.kP = AlgaePivotConstants.kP;
    talonFXSConfiguration.Slot0.kI = AlgaePivotConstants.kI;
    talonFXSConfiguration.Slot0.kD = AlgaePivotConstants.kD;
    talonFXSConfiguration.Slot0.kG = AlgaePivotConstants.kG;
    talonFXSConfiguration.Slot0.kV = AlgaePivotConstants.kV;
    talonFXSConfiguration.Slot0.kA = AlgaePivotConstants.kA;

    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
    talonFXSConfiguration.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.PulseWidth;
    talonFXSConfiguration.ExternalFeedback.QuadratureEdgesPerRotation = 2048;
    talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 100;
    talonFXSConfiguration.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 0.5;
    talonFXSConfiguration.ExternalFeedback.AbsoluteSensorOffset = 0.375372;
    talonFXSConfiguration.CurrentLimits.SupplyCurrentLimit = AlgaePivotConstants.supplyCurrentLimit;
    talonFXSConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
        AlgaePivotConstants.lowerSupplyCurrentLimit;
    talonFXSConfiguration.MotionMagic.MotionMagicAcceleration =
        AlgaePivotConstants.MotionMagicAcceleration;
    talonFXSConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        AlgaePivotConstants.MotionMagicCruiseVelocity;
    talonFXSConfiguration.MotionMagic.MotionMagicJerk = AlgaePivotConstants.MotionMagicJerk;
    talonFXSConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXSConfiguration.ExternalFeedback.SensorPhase = SensorPhaseValue.Opposed;

    m_AlgaePivotTalon = new TalonFXS(CANConstants.ALGAE_PIVOT, CANConstants.ELEVATOR_CANIVORE);
    m_AlgaePivotTalon.setNeutralMode(NeutralModeValue.Brake);
    m_AlgaePivotTalon.getConfigurator().apply(talonFXSConfiguration);

    m_AlgaePivotTalon.setControl(new DutyCycleOut(0));
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

  public boolean shouldGoToMidPoint() {
    if (hasAlgae.getAsBoolean() && !coralLevelSupplier.get().name().equals("PROCESSER")) {
      return true;
    }
    return false;
  }

  public Command goToLevel(CoralLevels level) {
    ConditionalCommand command =
        new ConditionalCommand(goToMidPoint(), rotateToLevel(level), this::shouldGoToMidPoint);

    return command;
  }

  public Command setAngle(CoralLevels level) {
    return runOnce(() -> rotate(level.algaeAngle));
  }

  public Command goToMidPoint() {
    return runOnce(
        () -> {
          rotate(AlgaePivotConstants.MIDPOINT);
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Pivot", m_AlgaePivotTalon.getPosition().getValueAsDouble());
  }

  private void rotate(Angle setpoint) {
    m_AlgaePivotTalon.setControl(m_motionMatiVoltage.withPosition(setpoint));
  }

  public Command openLoopCommandUp() {
    return runOnce(
        () -> {
          openLoop(0.1);
        });
  }

  public Command openLoopCommandDown() {
    return runOnce(
        () -> {
          openLoop(-0.1);
        });
  }

  private void openLoop(double power) {
    m_AlgaePivotTalon.setControl(new DutyCycleOut(power));
  }

  private Command rotateToLevel(CoralLevels level) {
    return runOnce(
        () -> {
          rotate(level.algaeAngle);
        });
  }

  public void disable() {
    m_AlgaePivotTalon.setControl(new DutyCycleOut(0));
  }
}
