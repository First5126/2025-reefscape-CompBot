// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.CoralLevels;
import frc.robot.constants.ElevatorConstants;
import java.util.Arrays;
import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANdi m_CANdi;
  private final MotionMagicVoltage m_moitonMagicVoltage;
  private final VoltageOut m_VoltageOut = new VoltageOut(0);
  private final Slot0Configs m_slot0Configs = new Slot0Configs();

  // Keep track of the current coral level designated for the elevator
  private CoralLevels[] m_corralLevels = {
    CoralLevels.L1, CoralLevels.L2, CoralLevels.L3, CoralLevels.L4
  };
  private CoralLevels m_currentCoralLevel = m_corralLevels[0];

  public Elevator() {
    m_leftMotor = new TalonFX(CANConstants.LEFT_ELAVOTAR_MOTOR, CANConstants.ELEVATOR_CANIVORE);
    m_rightMotor = new TalonFX(CANConstants.RIGHT_ELAVOTAR_MOTOR, CANConstants.ELEVATOR_CANIVORE);

    m_CANdi = new CANdi(CANConstants.ELEVATOR_CANDI, CANConstants.ELEVATOR_CANIVORE);

    TalonFXConfiguration m_leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration m_rightConfig = new TalonFXConfiguration();

    m_leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_leftConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;
    m_leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    m_slot0Configs.kP = ElevatorConstants.kP;
    m_slot0Configs.kI = ElevatorConstants.kI;
    m_slot0Configs.kD = ElevatorConstants.kD;
    m_slot0Configs.kG = ElevatorConstants.kG;
    m_slot0Configs.kV = ElevatorConstants.kV;
    m_slot0Configs.kS = ElevatorConstants.kS;

    m_leftConfig.Slot0 = m_slot0Configs;
    m_moitonMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);
    m_leftConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
    m_leftConfig.MotionMagic.MotionMagicAcceleration = 60;
    m_leftConfig.MotionMagic.MotionMagicJerk = 360;

    m_leftConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    m_leftConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = m_CANdi.getDeviceID();

    m_leftConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    m_leftConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_CANdi.getDeviceID();
    m_leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    m_leftConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

    m_leftConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    m_leftConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    m_leftMotor.getConfigurator().apply(m_leftConfig);
    m_rightMotor.getConfigurator().apply(m_rightConfig);
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));
    m_leftMotor.setControl(m_VoltageOut.withOutput(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height: ", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator Lower Limit Switch Status: ", isLowerLimitReached());
    SmartDashboard.putBoolean("Elevator Upper Limit Switch Status: ", isUpperLimitReached());
  }

  public Command openLoopCommand(Supplier<Double> speed) {
    return run(() -> setSpeed(speed.get()));
  }

  public Command lowerToNextPosition() {
    return changeLevelIncrement(-1);
  }

  public Command raiseToNextPosition() {
    return changeLevelIncrement(1);
  }

  /**
   * Slowely move the elevator in the up direction
   *
   * @return Command indication to move the elevator in the up direction
   */
  public Command trimUp() {
    return moveMotor(ElevatorConstants.TRIM_UP_SPEED);
  }

  /**
   * Slowly move the elevator in the down direction
   *
   * @return Command indicating to move the elevator in the down direction
   */
  public Command trimDown() {
    return moveMotor(ElevatorConstants.TRIM_DOWN_SPEED);
  }

  public Command goToTop() {
    return setCoralPosition(CoralLevels.L4);
  }

  public Command goToBottom() {
    return setCoralPosition(CoralLevels.L1);
  }

  public Command setCoralPosition(CoralLevels position) {
    return runOnce(
        () -> {
          setPosition(position);
        });
  }

  public Command stopMotors() {
    return runOnce(() -> m_leftMotor.setControl(m_VoltageOut.withOutput(0)));
  }

  /**
   * Move the elevator motor in a direction based on the supplied power
   *
   * @param power Double supplied indicating the direction to move the motor
   * @return Command indicating the ability to move the elevator motors in a designated direction
   */
  public Command moveMotor(Supplier<Double> power) {
    return run(
        () -> {
          setControl(new DutyCycleOut(power.get() * 0.1));
        });
  }

  public Command unBrake() {
    return run(
        () -> {
          MotorOutputConfigs unbrake = new MotorOutputConfigs();
          unbrake.NeutralMode = NeutralModeValue.Coast;

          m_leftMotor.getConfigurator().apply(unbrake);
          m_rightMotor.getConfigurator().apply(unbrake);
        });
  }

  public Command brake() {
    return run(
        () -> {
          MotorOutputConfigs brake = new MotorOutputConfigs();
          brake.NeutralMode = NeutralModeValue.Brake;

          m_leftMotor.getConfigurator().apply(brake);
          m_rightMotor.getConfigurator().apply(brake);
        });
  }

  public Command zeroElevator() {
    // TODO: add logic for zero elevator

    return Commands.none();
  }

  public Command disable() {
    return run(
        () -> {
          setControl(new DutyCycleOut(0));
        });
  }

  private double getElevatorHeight() {
    return m_leftMotor.getPosition().getValue().in(Rotations);
  }

  private void setSpeed(double speed) {
    setControl(m_VoltageOut.withOutput(speed * 12.0));
  }

  private void setControl(ControlRequest control) {
    m_leftMotor.setControl(control);
  }

  private Command changeLevelIncrement(int change) {
    int newIndex = currentCoralIndex() + change;

    if (newIndex < 0) {
      newIndex = 0;
    } else if (newIndex > m_corralLevels.length - 1) {
      newIndex = m_corralLevels.length - 1;
    }
    return setCoralPosition(m_corralLevels[newIndex]);
  }

  // using exesting mPositionVoltage write set position method in meters
  private void setPosition(CoralLevels position) {
    m_currentCoralLevel = position;
    m_leftMotor.setControl(m_moitonMagicVoltage.withPosition(position.heightAngle));
    SmartDashboard.putNumber("goal position", position.heightAngle.in(Revolutions));
  }

  private int currentCoralIndex() {
    int index = Arrays.binarySearch(m_corralLevels, m_currentCoralLevel);
    // if not found, make sure return bottom
    return (index < 0) ? 0 : index;
  }

  private boolean isLowerLimitReached() {
    return m_leftMotor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
  }

  private boolean isUpperLimitReached() {
    return m_leftMotor.getForwardLimit().getValue().equals(ForwardLimitValue.ClosedToGround);
  }
}
