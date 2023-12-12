// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Manipulator;

import java.util.Timer;

/** An example command that uses an example subsystem. */
public class rohanIntakeCube extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_manipulatorMotor;
  private Timer timer;

  public rohanIntakeCube(Manipulator mManipulatorMotor) {
    m_manipulatorMotor = mManipulatorMotor;
    addRequirements(mManipulatorMotor);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulatorMotor.intakeCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulatorMotor.stopCube();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
