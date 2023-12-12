// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RafaelWheel extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_motor;

  public RafaelWheel() {
    m_motor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    m_motor.configFactoryDefault();
    m_motor.setInverted(false);
  }

  public void runMotor(){
    m_motor.set(ControlMode.PercentOutput, -0.5);
  }
  public void stopMotor(){
    m_motor.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
