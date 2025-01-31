// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Dpaddetection extends SubsystemBase {
  private CommandXboxController m_driverController;
  private CommandXboxController m_coDriverController;

  private String m_positionSelection;
  private boolean positionEnabled = false;
  

  public Dpaddetection (CommandXboxController driverController, CommandXboxController copilotController) {
    m_driverController = driverController;
    m_coDriverController = copilotController;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command StartRecord() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(
        () -> {
             
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //When back button is pressed...
    if (positionEnabled) {
      switch (m_driverController.getHID().getPOV()) {
        case 0:
         m_positionSelection = "far";
          break;
        case 90:
         if (m_positionSelection.equals("far")){
          m_positionSelection = "far right";
         } else if (m_positionSelection.equals("near")) {
          m_positionSelection = "near right";
         }
          break;
        case 180:
         m_positionSelection = "near";
          break;
        case 270:
        if (m_positionSelection.equals("far")){
          m_positionSelection = "far left";
         } else if (m_positionSelection.equals("near")) {
          m_positionSelection = "near left";
         }
        
          break;
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}