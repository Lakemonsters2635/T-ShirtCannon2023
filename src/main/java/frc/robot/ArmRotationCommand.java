// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmRotationCommand extends CommandBase {
  /** Creates a new ArmRotationCommand. */
  private RotarySubsystem m_RotarySubsystem;

  public ArmRotationCommand(RotarySubsystem rotarySubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    m_RotarySubsystem = rotarySubsystem;

    addRequirements(m_RotarySubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RotarySubsystem.resetEncoderCounts();
    System.out.println("This command runs plsplsplspls");
    m_RotarySubsystem.rotate();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    System.out.println("the command is running");
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RotarySubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_RotarySubsystem.getEncoderCounts());
    return false;
  }
}
