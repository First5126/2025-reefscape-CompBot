package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CANConstants;

public class AlgaeRollers extends SubsystemBase {
  private TalonFX m_motorOne;

  private Trigger m_hasGamePiece;

  private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

  public AlgaeRollers() {
    m_hasGamePiece = new Trigger(this::getGamePieceDetected);
    m_motorOne = new TalonFX(CANConstants.ALGAE_MOTOR, CANConstants.ELEVATOR_CANIVORE);
  }

  private boolean getGamePieceDetected() {
    return m_motorOne.getFault_ForwardSoftLimit().getValue();
  }

  public Trigger hasGamePiece() {
    return m_hasGamePiece;
  }

  public Command feedIn() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(AlgaeConstants.INTAKE_SPEED));
        });
  }

  public Command feedOut() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(AlgaeConstants.OUTTAKE_SPEED));
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorOne.setControl(m_velocityVoltage.withVelocity(0.0));
        });
  }

  public void periodic() {
    // SmartDashboard.putString("Algae Motor Rotation", m_motorOne.getDescription());
  }
}
