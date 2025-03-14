// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Deleted Elevator branch

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RecordInputs extends SubsystemBase {

  private String m_positionSelection = "";
  private boolean positionEnabled = false;
  private String m_selectedCoralStationSide = "left";

  public Command setFar() {
    return Commands.runOnce(
        () -> {
          m_positionSelection = "far";
        });
  }

  public Command setNear() {
    return Commands.runOnce(
        () -> {
          m_positionSelection = "near";
        });
  }

  public Command setRight() {
    return Commands.runOnce(
        () -> {
          if (positionEnabled) {
            if (m_positionSelection == "far") {
              m_positionSelection = "far right";
            } else {
              m_positionSelection = "near right";
            }
          }
        });
  }

  public Command setLeft() {
    return Commands.runOnce(
        () -> {
          if (positionEnabled) {
            if (m_positionSelection == "far") {
              m_positionSelection = "far left";
            } else {
              m_positionSelection = "near left";
            }
          }
        });
  }

  public Command setEnabled() {
    return Commands.runOnce(
        () -> {
          positionEnabled = !positionEnabled;
          m_positionSelection = "";
        });
  }

  public Command setLeftSideCoralStation() {
    return runOnce(
        () -> {
          m_selectedCoralStationSide = "left";
        });
  }

  public Command setRightSideCoralStation() {
    return runOnce(
        () -> {
          m_selectedCoralStationSide = "right";
        });
  }

  public boolean rightCoralStationSelected() {
    if (m_selectedCoralStationSide.equals("right")) {
      return true;
    }
    return false;
  }

  public boolean leftCoralStationSelected() {
    if (m_selectedCoralStationSide.equals("left")) {
      return true;
    }
    return false;
  }

  // returns the selected side of the coral station i.e. the left or right side of the coral station
  public String getSelectedCoralStationSide() {
    return m_selectedCoralStationSide;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Position enabled", positionEnabled);
    SmartDashboard.putString("Position Selection", m_positionSelection);
    SmartDashboard.putString("Coral Side Selected", m_selectedCoralStationSide);
  }
}
