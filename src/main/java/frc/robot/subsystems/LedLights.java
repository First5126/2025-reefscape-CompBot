// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.CANConstants;



public class LedLights extends SubsystemBase {
  private CANdle m_candle = new CANdle(CANConstants.CANDLE_ID, Constants.CANIVORE_BUS_NAME);

  public LedLights() {}

  /**
   * Apply color to full set of LEDs on the robot
   *
   * @return a Command for applying a color
   */
  public Command applyColor(int r, int g, int b) {
    return runOnce(
      () -> {
        m_candle.setLEDs(r,g,b);
      });
  }

  public Command lightBowCommand() {
    m_candle.setLEDs(0,255,0,0,2,10);
        return null;
        }
    
      private void setLEDs() {
        
        throw new UnsupportedOperationException("'setLEDs'");
      }
    
      private void setAnimation(Animation animation){
       m_candle.animate(animation);
      }
  }
