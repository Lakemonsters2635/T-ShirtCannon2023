// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DifferentialDrive2;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ShooterCommand;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive2 m_tankDrive = new DifferentialDrive2(); 
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterCommand shooterCommand = new ShooterCommand(m_shooterSubsystem);

  private Joystick m_leftStick;
  private Joystick m_rightStick;
  // private CommandXboxController m_controller;
  // private XboxController m_controller;

  private Trigger shooterButton;

  private Timer timer;

  // Trigger shooterButton = new JoystickButton(m_controller, XboxController.Button.kB.value);


  @Override
  public void robotInit() {
    
    m_leftStick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
    m_rightStick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
    // m_controller = new XboxController(Constants.CONTROLLER_CHANNEL);
    shooterButton = new JoystickButton(m_leftStick, 1);
    shooterButton.onTrue(shooterCommand);
    timer = new Timer();
  }

  @Override
  public void teleopPeriodic() {
    m_tankDrive.drive(m_leftStick.getY(), m_rightStick.getY());
    
    // m_tankDrive.drive(m_controller.getLeftY()/2, m_controller.getRightY()/2);
    // shooterButton.onTrue(shooterCommand);

    // ALL OF THESE IF STATEMENTS ARE FOR TESTING THE RELAY STATES

    // if(m_leftStick.getY() > 0.1){  
    //   m_shooterSubsystem.shootForward(); // red
    // }
    // else if (m_leftStick.getY() < -0.1) {
    //   m_shooterSubsystem.shootBack(); // green
    // }
    // if(m_rightStick.getY() > 0.1){
    //   m_shooterSubsystem.shootOn(); // orange
    // }
    // else if (m_rightStick.getY() < -0.1) {
    //   m_shooterSubsystem.shootOff(); // no light
    // }

    /*  THIS IS THE ACTUAL TEST CODE FOR LAUNCHING THINGS :D
    When you press the trigger, it sets RELAY to the open state and after 0.5 seconds 
    and then sets it closed. (FIY: WE DON'T KNOW WHICH RELAY VALUES ARE CORRECT AND ACTUALLY WORK 
    SO WE NEED TO TEST EACH VALUE WITH THE GAS TANKFULL)
    */
    if (m_rightStick.getTriggerPressed()) {
      timer.start();
      shooterCommand.initialize();
    }
    if (timer.get() >= 0.5) {
      shooterCommand.end(true);
      timer.stop();
      timer.reset();
      // System.out.println(timer.get());
    }
    
   
    
  }
}
