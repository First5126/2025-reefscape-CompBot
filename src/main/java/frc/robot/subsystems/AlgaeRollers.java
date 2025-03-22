package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CANConstants;
import frc.robot.subsystems.LedLights.RobotState;

public class AlgaeRollers extends SubsystemBase {
  private TalonFXS m_motorOne;

  private Trigger m_hasGamePiece;
  private LedLights m_ledLights = LedLights.getInstance();

  private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public AlgaeRollers() {
    m_motorOne = new TalonFXS(CANConstants.ALGAE_MOTOR, CANConstants.ELEVATOR_CANIVORE);
    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXSConfiguration.Slot0.kP = AlgaeConstants.kP;
    talonFXSConfiguration.Slot0.kI = AlgaeConstants.kI;
    talonFXSConfiguration.Slot0.kD = AlgaeConstants.kD;
    talonFXSConfiguration.Slot0.kG = AlgaeConstants.kG;
    talonFXSConfiguration.Slot0.kA = AlgaeConstants.kA;
    talonFXSConfiguration.Slot0.kV = AlgaeConstants.kV;

    talonFXSConfiguration.HardwareLimitSwitch.ForwardLimitSource =
        ForwardLimitSourceValue.LimitSwitchPin;
    talonFXSConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;

    m_motorOne.getConfigurator().apply(talonFXSConfiguration);

    m_hasGamePiece =
        new Trigger(this::isAlgaeLoaded).debounce(AlgaeConstants.DEBOUNCE, DebounceType.kFalling);
    m_ledLights.registerTrigger(m_hasGamePiece, RobotState.ALGAE_RECEIVED);
  }

  public Trigger hasAlgae() {
    return m_hasGamePiece;
  }

  public Command startFeedIn() {
    return run(
        () -> {
          m_motorOne.setControl(new VoltageOut(AlgaeConstants.INTAKE_SPEED));
        });
  }

  public Command feedIn() {
    return m_ledLights
        .applyState(RobotState.ALGAE_INTAKE)
        .andThen(startFeedIn().until(m_hasGamePiece).andThen(holdAlgae()));
  }

  public Command startFeedOut() {
    return run(
        () -> {
          m_motorOne.setControl(new VoltageOut(AlgaeConstants.OUTTAKE_SPEED));
        });
  }

  public Command feedOut() {
    return startFeedOut().onlyWhile(m_hasGamePiece).andThen(stop());
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorOne.setControl(new VoltageOut(0));
        });
  }

  public Command holdAlgae() {
    return runOnce(
        () -> {
          m_motorOne.setControl(new VoltageOut(AlgaeConstants.HOLDING_SPEED));
        });
  }

  public void periodic() {
    SmartDashboard.putBoolean("Has Algae", isAlgaeLoaded());
    SmartDashboard.putNumber("Algae Intake Speed", m_motorOne.getVelocity().getValueAsDouble());
    // SmartDashboard.putString("Algae Motor Rotation", m_motorOne.getDescription());
  }

  private boolean isAlgaeLoaded() {
    return m_motorOne.getForwardLimit().getValue().value == 0;
  }

  public void disable() {
    m_motorOne.setControl(new DutyCycleOut(0));
  }
}
