// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class muneerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_muneerMotor;
  private double intakeCubeSpeed = -0.4;
  private double outtakeCubeSpeed = 0.6;
  /** Creates a new ExampleSubsystem. */
  public muneerSubsystem() {
    m_muneerMotor = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    m_muneerMotor.configFactoryDefault();
    m_muneerMotor.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void intakeCube() {
    m_muneerMotor.set(ControlMode.PercentOutput, intakeCubeSpeed);
  }

  public void outtakeCube() {
    m_muneerMotor.set(ControlMode.PercentOutput, outtakeCubeSpeed);
  }

  public void StopManipulator() {
    m_muneerMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getSpeedIntakeCube() {
    return intakeCubeSpeed;
  }

  public double getSpeedouttakeCube() {
    return outtakeCubeSpeed;
  }

  public void setSpeedIntakeCube(double x) {
    intakeCubeSpeed = x;
  }

  public void setSpeedouttakeCube(double x) {
    outtakeCubeSpeed = x;
  }


  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
