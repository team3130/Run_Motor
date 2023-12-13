// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UmarWheel extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final WPI_TalonSRX m_motor;
  public double speedIntakeCube = -0.5;

  public UmarWheel() {
    m_motor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    m_motor.configFactoryDefault();
    m_motor.setInverted(false);
  }

  public void runMotor(){
    m_motor.set(ControlMode.PercentOutput,speedIntakeCube);
  }

  public void stopMotor(){
    m_motor.set(ControlMode.PercentOutput,0);
  }

  public double getSpeedIntakeCube(){
    return speedIntakeCube;
  }

  public void setSpeedIntakeCube(double newSpeedIntakeCube){
    this.speedIntakeCube = newSpeedIntakeCube;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Motor Speed: Intake Cube", this::getSpeedIntakeCube, this::setSpeedIntakeCube);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
