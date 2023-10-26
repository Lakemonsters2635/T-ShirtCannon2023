// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Encoder;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  public static XboxController m_Controller;
  public static RotarySubsystem m_RotarySubsystem = new RotarySubsystem();
  public static ArmRotationCommand m_ArmRotationCommand = new ArmRotationCommand(m_RotarySubsystem);
  
  
  @Override
  public void robotInit(){
    m_Controller  = new XboxController(Constants.CONTROLLER_CHANNEL);
    
    //m_ArmRotationCommand.execute();
  }

  @Override
  public void robotPeriodic() {
   
    Trigger rotationButton = new JoystickButton(m_Controller, XboxController.Button.kLeftBumper.value);
    rotationButton.onTrue(m_ArmRotationCommand);
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
  public void teleopPeriodic() {
   
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
