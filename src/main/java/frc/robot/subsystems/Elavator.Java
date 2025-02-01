// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.simulation.TalonFXSimCollection;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final TalonFX m_leftMotor = new TalonFX(0);
  private final TalonFX m_rightMotor = new TalonFX(1);
  private final Follower m_rightFollow = new Follower(0, true);
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0).withFeedForward(0);
  private final VoltageOut m_VoltageOut = new VoltageOut(0);
  
  private DCMotor m_motor = DCMotor.getKrakenX60(2).withReduction(24);
  private LinearSystem<N2,N1,N1>  m_elevatorPlant; 
  private ElevatorSim m_simulator ;
  

  public Elevator() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = 24.0;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    m_leftMotor.getConfigurator().apply(leftConfig);
    m_rightMotor.getConfigurator().apply(rightConfig);
    m_rightMotor.setControl(m_rightFollow);
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    //simulation
    m_elevatorPlant = LinearSystemId.createElevatorSystem(m_motor, 5.0, 0.05, 1.0);
    m_simulator = new ElevatorSim(m_elevatorPlant, m_motor, 0.20, 1.2, true, 0.20);

  }
 
  public void notifyThis(){
    System.out.println("notified");
  }
  public double getElevatorHeight(){
    return m_leftMotor.getRotorPosition().getValue() / 24.0 * 2.0 * Math.PI * 0.05;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height: " , getElevatorHeight());
    SmartDashboard.putNumber("SimElevator Height: " , m_simulator.getPositionMeters());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of our elevator.
    // We set the inputs of the elevator plant to the voltages, and update the simulation.
    m_simulator.setInput(m_leftMotor.getMotorVoltage().getValue());
    m_simulator.update(0.020);// 20 ms per update, 50 Hz update rate
    // We then set the simulated encoder distance and velocity in the elevator object.
    TalonFXSimState simState = m_leftMotor.getSimState();
    simState.setSupplyVoltage(m_simulator.getPositionMeters());
    simState.setRotorVelocity(m_simulator.getVelocityMetersPerSecond());
  }

  private void setSpeed(double speed){
    setControl(m_VoltageOut.withOutput(speed * 12.0));
  }

  private void setControl(ControlRequest control){
    m_leftMotor.setControl(control);
  }

  public Command openLoopCommand(Supplier<Double> speed) {
    return run(() -> setSpeed(speed.get()));
  }


  //using exesting mPositionVoltage write set position method in meters
  public Command setPosition(double position){
    return run(() -> setControl(m_PositionVoltage.withPosition(position)));
  }
}