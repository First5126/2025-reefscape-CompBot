// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.CoralLevels;


public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor = new TalonFX(CANConstants.LEFT_ELAVOTAR_MOTOR);
  private final TalonFX m_rightMotor = new TalonFX(CANConstants.RIGHT_ELAVOTAR_MOTOR);

  private final CANdi m_CANdi = new CANdi(CANConstants.ELEVATOR_CANDI);
  private final MotionMagicVoltage m_moitonMagicVoltage;
  private final VoltageOut m_VoltageOut = new VoltageOut(0);  
  private final Slot0Configs m_slot0Configs = new Slot0Configs();

  // These fields are for when the driver taps up or down on the dpad. The elvator will go up or down a whole coral level
  private int m_goalHeightIndex = 0;
  
  public Elevator() {    
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    m_slot0Configs.kP = ElevatorConstants.kP;
    m_slot0Configs.kI = ElevatorConstants.kI;
    m_slot0Configs.kD = ElevatorConstants.kD;
    m_slot0Configs.kG = ElevatorConstants.kG;
    m_slot0Configs.kV = ElevatorConstants.kV;
    m_slot0Configs.kS = ElevatorConstants.kS;

    leftConfig.Slot0 = m_slot0Configs;
    m_moitonMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
    leftConfig.MotionMagic.MotionMagicAcceleration = 80;
    leftConfig.MotionMagic.MotionMagicJerk = 400;

    leftConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    leftConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = m_CANdi.getDeviceID();

    leftConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    leftConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_CANdi.getDeviceID();
    leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =  0.0;
    leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

    leftConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    leftConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));
    m_leftMotor.setControl(m_VoltageOut.withOutput(0));
  }

  public double getElevatorHeight(){
    return m_leftMotor.getPosition().getValueAsDouble() / 24.0 * 2.0 * Math.PI * 0.05;//24:1 gear ratio, 2" diameter pulley
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height: ", getElevatorHeight());
  }

  private void setSpeed(double speed){
    setControl(m_VoltageOut.withOutput(speed * 12.0));
  }

  private void setControl(ControlRequest control){
    m_leftMotor.setControl(control);
  }

  public Command openLoopCommand(Supplier<Double> speed) {
    return run(() -> setSpeed(speed.get()));
  }

  private void changeGoalHeightIndex(int change) {
    m_goalHeightIndex += change;

    if (m_goalHeightIndex<0) m_goalHeightIndex = 0;
    if (m_goalHeightIndex>CoralLevels.values().length-1) m_goalHeightIndex = CoralLevels.values().length-1;
    setPosition(CoralLevels.values()[m_goalHeightIndex]);
  }

  public Command lowerElevator() {
    return runOnce(() -> {
      changeGoalHeightIndex(-1);
    });
  }

  public Command raiseElevator() {
    return runOnce(() -> {
      changeGoalHeightIndex(1);
    });
  }

  public Command goToTop() {
    return runOnce(() -> {
      changeGoalHeightIndex(5);
    });
  }

  public Command goToBottom() {
    return runOnce(() -> {
      changeGoalHeightIndex(-5);
    });
  }

  private boolean getIsAtPosition() {
    return m_leftMotor.getPosition().getValue().isNear(ElevatorConstants.CoralLevels.values()[m_goalHeightIndex].heightAngle, ElevatorConstants.ELEVATOR_READING_STDV);
  }

  //using exesting mPositionVoltage write set position method in meters
  private void setPosition(CoralLevels position){
    m_leftMotor.setControl(m_moitonMagicVoltage.withPosition(position.heightAngle));
  }

  public Command goToCoralHeightPosition(CoralLevels position) {
    return runOnce(
        ()-> {
          setPosition(position);
        });
  }

  public Command stopMotors(){
    return runOnce(
        () -> m_leftMotor.setControl(m_VoltageOut.withOutput(0)
      ));
  }

  public Command moveMotor(Supplier<Double> power) {
    return run (
      () -> {
        setControl(new DutyCycleOut(power.get()*-0.1));
      });
  }

  public Command unBrake() {
    return run (
      () -> {
        MotorOutputConfigs unbrake = new MotorOutputConfigs();
        unbrake.NeutralMode = NeutralModeValue.Coast;

        m_leftMotor.getConfigurator().apply(unbrake);
        m_rightMotor.getConfigurator().apply(unbrake);
      });
  }

  public Command brake() {
    return run (
      () -> {
        MotorOutputConfigs brake = new MotorOutputConfigs();
        brake.NeutralMode = NeutralModeValue.Brake;

        m_leftMotor.getConfigurator().apply(brake);
        m_rightMotor.getConfigurator().apply(brake);
      });
  }
}
