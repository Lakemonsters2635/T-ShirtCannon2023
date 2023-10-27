// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooterSubsystem;
  Timer m_timer;

  double startTime;
  double endTime;
  double delay = 0.5; // How long we want the relay to be set to kForward

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_timer = new Timer();
    
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    startTime = m_timer.get();
    m_shooterSubsystem.shoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endTime = m_timer.get();
    System.out.println("Shootercommand is working");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endTime-startTime > delay) {
      m_shooterSubsystem.stop();
      return true;
    }
    return false;
  }
}
