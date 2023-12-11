// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensionarm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtensionArm;

/** An example command that uses an example subsystem. */
public class ExtensionExtend extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_ExtensionArm;
  private final RobotContainer m_robotContainer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtensionExtend(ExtensionArm subsystem, RobotContainer container) {
    m_ExtensionArm = subsystem;
    m_robotContainer = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ExtensionArm.LimitSwitch()){
      m_ExtensionArm.resetEncoders();
      m_ExtensionArm.setZeroed();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ExtensionArm.inSlowZone()) { // if arm is in slow zone, then get value of joystick and set speed based on adjusted joystick value
      double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1);
      m_ExtensionArm.runMotor(m_ExtensionArm.rawMotorSpeed(y) * m_ExtensionArm.slowZoneFactor());
    }
    else {
      double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1); // if arm is not in slow zone, then get value of joystick and set speed based on joystick value
      m_ExtensionArm.runMotor(m_ExtensionArm.rawMotorSpeed(y));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


