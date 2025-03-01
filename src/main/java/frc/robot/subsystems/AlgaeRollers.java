package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CANConstants;

public class AlgaeRollers extends SubsystemBase {
  private TalonFXS m_motorOne;

  private CANrange m_algaeCANrange;

  private Trigger m_hasGamePiece;

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

    m_motorOne.getConfigurator().apply(talonFXSConfiguration);

    CANrangeConfiguration CANrangeConfiguration = new CANrangeConfiguration();
    CANrangeConfiguration.ProximityParams.ProximityThreshold = AlgaeConstants.PROXIMITY_THRESHOLD;

    m_algaeCANrange = new CANrange(CANConstants.ALGAE_CAN_RANGE, CANConstants.ELEVATOR_CANIVORE);
    m_algaeCANrange.getConfigurator().apply(CANrangeConfiguration);

    m_hasGamePiece = new Trigger(this::getGamePieceDetected);
  }

  private boolean getGamePieceDetected() {
    return m_algaeCANrange.getIsDetected().getValue();
  }

  public Trigger hasAlgae() {
    return m_hasGamePiece;
  }

  public Command feedIn() {
    return runOnce(
            () -> {
              m_motorOne.setControl(new VoltageOut(AlgaeConstants.INTAKE_SPEED));
            })
        .until(hasAlgae())
        .andThen(holdAlgae().onlyWhile(hasAlgae()).andThen(stop()));
  }

  public Command feedOut() {
    return runOnce(
            () -> {
              m_motorOne.setControl(new VoltageOut(AlgaeConstants.OUTTAKE_SPEED));
            })
        .onlyWhile(hasAlgae())
        .andThen(stop());
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(0.0));
        });
  }

  public Command holdAlgae() {
    return runOnce(
        () -> {
          m_motorOne.setControl(new VoltageOut(AlgaeConstants.HOLDING_SPEED));
        });
  }

  public void periodic() {
    // SmartDashboard.putString("Algae Motor Rotation", m_motorOne.getDescription());
  }
}
