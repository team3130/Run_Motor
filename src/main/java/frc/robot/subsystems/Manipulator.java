// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  private final WPI_TalonSRX m_manipulatorMotor; // we should probably change these names once we learn more
  private double intakeConeSpeed = .5;
  private double outtakeConeSpeed = -.5;
  private double intakeCubeSpeed = -.5;
  private double outtakeCubeSpeed = .5;

  public Manipulator() {
    m_manipulatorMotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    m_manipulatorMotor.configFactoryDefault();
    m_manipulatorMotor.setInverted(false);
  }

  public void intakeCone() {
      m_manipulatorMotor.set(ControlMode.PercentOutput, intakeConeSpeed);
  }

  public void outtakeCone() {
      m_manipulatorMotor.set(ControlMode.PercentOutput, outtakeConeSpeed);
  }

  public void intakeCube(){m_manipulatorMotor.set(ControlMode.PercentOutput, intakeCubeSpeed);}

  public void outtakeCube(){m_manipulatorMotor.set(ControlMode.PercentOutput, outtakeCubeSpeed);}

  public double getSpeedIntakeCone() {
      return intakeConeSpeed;
  }
  public double getSpeedOuttakeCone() {
      return outtakeConeSpeed;
  }
  public double getSpeedIntakeCube(){return intakeCubeSpeed;}
  public double getSpeedOuttakeCube(){return outtakeCubeSpeed;}



  public void setSpeedOuttakeCone(double x) {
      outtakeConeSpeed = x;
  }
  public void setSpeedIntakeCone(double x){intakeConeSpeed = x;}
  public void setSpeedOuttakeCube(double x){outtakeCubeSpeed = x;}
  public void setSpeedIntakeCube(double x){intakeCubeSpeed = x;}

  public void StopManipulator() {
    m_manipulatorMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {

  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Motor Speed: Intake Cone", this::getSpeedIntakeCone, this::setSpeedIntakeCone);
    builder.addDoubleProperty("Motor Speed: Outtake Cone", this::getSpeedOuttakeCone, this::setSpeedOuttakeCone);
    builder.addDoubleProperty("Motor Speed: Intake Cube", this::getSpeedIntakeCube, this::setSpeedIntakeCube);
    builder.addDoubleProperty("Motor Speed: Outtake Cube", this::getSpeedOuttakeCube, this::setSpeedOuttakeCube);
    
  }
}
