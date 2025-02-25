// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.CANConstants;

public class LedLights extends SubsystemBase {
  private CANdle m_candle = new CANdle(CANConstants.CANDLE_ID, Constants.CANIVORE_BUS_NAME);

  private static final Color8Bit CLEAR = new Color8Bit(0, 0, 0);
  private static final Color8Bit RED = new Color8Bit(255, 0, 0);
  private static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  private static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
  private static final Color8Bit ORANGE = new Color8Bit(255, 157, 0);
  private static final Color8Bit PURPLE = new Color8Bit(151, 0, 180);

  private static LedLights m_instance;

  public enum RobotState {
    EMPTY, // state when the robot is completly empty of any coral or algae
    CORAL_INTAKE_LEFT, // state when the robot is awaiting to intake coral on the left
    CORAL_INTAKE_RIGHT, // state when the robot is awaiting to intake coral on the right
    CORAL_RECEIVED, // state when robot has received coral for placing
    ALGAE_INTAKE, // state when the robot is performing an algae intake
    ALGAE_RECEIVED, // state when the robot has received algae and is ready for next step
    PLACING_CORAL_L1, // state when robot has determined to place on reef L1
    PLACING_CORAL_L2, // state when robot has determined to place on reef L2
    PLACING_CORAL_L3, // state when robot has determined to place on reef L3
    PLACING_CORAL_L4, // state when robot has determined to place on reef L4
    PROCESSING_ALGAE, // state when robot is processing the algae in the processor
    SHOOTING_ALGAE // state when robot is shooting algae
  }

  private LedLights() {}

  public static LedLights getInstance() {
    return m_instance == null ? m_instance = new LedLights() : m_instance;
  }

  /**
   * Apply color to full set of LEDs on the robot
   *
   * @return a Command for applying a color
   */
  public Command applyColor(Color8Bit color) {
    return runOnce(
        () -> {
          m_candle.setLEDs(color.red, color.green, color.blue);
        });
  }

  // TODO: define states
  public void setState(RobotState state) {
    switch (state) {
      case EMPTY:
        break;
      case CORAL_INTAKE_LEFT:
        break;
      case CORAL_INTAKE_RIGHT:
        break;

      default:
        break;
    }
  }

  public Command lightBowCommand() {
    m_candle.setLEDs(0, 255, 0, 0, 2, 10);
    return null;
  }

  private void setAnimation(Animation animation) {
    m_candle.animate(animation);
  }
}
