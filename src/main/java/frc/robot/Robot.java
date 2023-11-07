// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  public static Joystick leftJoystick = new Joystick(Constants.leftJoystick);
  public static Joystick rightJoystick = new Joystick(Constants.rightJoystick);
  public static RotarySubsystem m_RotarySubsystem = new RotarySubsystem();
  public static ArmRotationCommand m_ArmRotationCommand = new ArmRotationCommand(m_RotarySubsystem);
  
  public void configureBindings(){
    Trigger rotationButton = new JoystickButton(rightJoystick, Constants.ROTATION_BUTTON);
    rotationButton.onTrue(new ArmRotationCommand(m_RotarySubsystem));
    System.out.println("Robot.configureBindigs()");
  }
  
  
  @Override
  public void robotInit(){
    System.out.println("Robot.robotInit()");
    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    System.out.println(m_RotarySubsystem.getEncoderCounts());
    if(rightJoystick.getTriggerPressed()){
      m_ArmRotationCommand.initialize();
    }
    if(m_RotarySubsystem.getEncoderCounts()>=2550){
        m_ArmRotationCommand.end(true);
        m_RotarySubsystem.resetEncoderCounts();
    }
    if(leftJoystick.getTriggerPressed()){
      m_ArmRotationCommand.end(true);
    }
    
   // System.out.println("On true area");
  }


  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
