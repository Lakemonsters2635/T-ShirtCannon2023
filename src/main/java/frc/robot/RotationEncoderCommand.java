// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotationEncoderCommand extends CommandBase {
  private RotatorEncoderSubsystem m_RotatorEncoderSubsystem;
  /** Creates a new RotationEncoderCommandSubway. */
  public RotationEncoderCommand(RotatorEncoderSubsystem rotatorEncoderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RotatorEncoderSubsystem = rotatorEncoderSubsystem;

    addRequirements(rotatorEncoderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RotatorEncoderSubsystem.getEncoderCounts();
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
