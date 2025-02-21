package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ClimbingConstants;

public class Climbing extends SubsystemBase {

  private final TalonFX m_leftMotor = new TalonFX(CANConstants.LEFT_CLIMBING_MOTOR);
  private final TalonFX m_rightMotor = new TalonFX(CANConstants.RIGHT_CLIMBING_MOTOR);
  private final PositionVoltage m_positionVoltage =
      new PositionVoltage(0).withSlot(0).withFeedForward(0);
  private final Slot0Configs m_slot0Configs = new Slot0Configs();

  public Climbing() {

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    m_slot0Configs.kP = ClimbingConstants.kP;
    m_slot0Configs.kI = ClimbingConstants.kI;
    m_slot0Configs.kD = ClimbingConstants.kD;
    m_slot0Configs.kG = ClimbingConstants.kG;
    m_slot0Configs.kV = ClimbingConstants.kV;
    m_slot0Configs.kS = ClimbingConstants.kS;

    leftConfig.Slot0 = m_slot0Configs;

    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), true));

    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftMotor.setControl(new DutyCycleOut(0));

    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
  }

  public Command climb() {
    return run(() -> setPosition(ClimbingConstants.ROTATIONS_FOR_CLIMB));
  }

  public Command unClimb() {
    return run(() -> setPosition(0));
  }

  public Command zeroClimbing() {
    return run(() -> m_leftMotor.setPosition(0));
  }

  public void setPosition(double position) {
    m_leftMotor.setControl(m_positionVoltage.withPosition(position));
  }
}
