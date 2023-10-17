// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.robot.subsystems.DifferentialDrive2;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive2 m_tankDrive;
  // private Joystick m_leftStick;
  // private Joystick m_rightStick;
  private XboxController m_controller;

  @Override
  public void robotInit() {
    m_tankDrive = new DifferentialDrive2();
    // m_leftStick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
    // m_rightStick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
    m_controller = new XboxController(Constants.CONTROLLER_CHANNEL);
  }

  @Override
  public void teleopPeriodic() {
    // m_tankDrive.drive(-m_leftStick.getY(), -m_rightStick.getY());
    m_tankDrive.drive(m_controller.getLeftY()/2, m_controller.getRightY()/2);
  }
}
