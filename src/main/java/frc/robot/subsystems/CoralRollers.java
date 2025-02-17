// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.CoralConstants;

public class CoralRollers extends SubsystemBase {

    private TalonFXS m_coralTalonFXS;
    private VelocityVoltage m_VelocityVoltage;

    private CANrange m_LeftCANrange;
    private CANrange m_RightCANrange;

    private Trigger m_hasGamePiece;

    public CoralRollers() {
        m_VelocityVoltage = new VelocityVoltage(0);

        m_coralTalonFXS = new TalonFXS(0);
        m_coralTalonFXS.setControl(new DutyCycleOut(0));

        CANrangeConfiguration CANrangeConfiguration = new CANrangeConfiguration();
        CANrangeConfiguration.ProximityParams.ProximityThreshold = CoralConstants.PROXIMITY_THRESHOLD;

        m_LeftCANrange = new CANrange(CANConstants.LEFT_CAN_RANGE_CORAL);
        m_LeftCANrange.getConfigurator().apply(CANrangeConfiguration);

        m_RightCANrange = new CANrange(CANConstants.RIGHT_CAN_RANGE_CORAL);
        m_RightCANrange.getConfigurator().apply(CANrangeConfiguration);

        m_hasGamePiece = new Trigger(this::isDetected).debounce(CoralConstants.DEBOUNCE);
    }

    public Trigger getCoralTrigger() {
        return m_hasGamePiece;
    }

    private boolean isDetected() {
        return m_LeftCANrange.getIsDetected().getValue() || m_RightCANrange.getIsDetected().getValue();
    }

    private void runRollers(double speed){
        m_coralTalonFXS.set(speed);
    }
    
    public Command setRollerSpeed(double speed) {
        return runOnce(
            () -> {
                runRollers(speed);
            });
    }
    private void rollIn() {
        m_coralTalonFXS.setControl(m_VelocityVoltage.withVelocity(CoralConstants.INTAKE_SPEED));
    }
    public Command rollInCommand() {
            return run(
                () -> {
                    rollIn();
                }).until(m_hasGamePiece).andThen(stopCommand());
    }
    private void rollOut() {
        m_coralTalonFXS.setControl(m_VelocityVoltage.withVelocity(CoralConstants.OUTTAKE_SPEED));
    }
    public Command rollOutCommand() {
            return run(
                () -> {
                    rollOut();
                }).onlyWhile(m_hasGamePiece).andThen(stopCommand());
    }
    private void stop() {
        m_coralTalonFXS.setControl(new DutyCycleOut(0));
    }
    public Command stopCommand() {
            return runOnce(
                () -> {
                    stop();
                });
    }
} 
