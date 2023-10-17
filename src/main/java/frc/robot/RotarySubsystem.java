// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RotarySubsystem extends SubsystemBase{
  /** Creates a new RotarySubsystem. */
  public Talon rotaryMotor; // Instantiates rotating motor

  public RotarySubsystem(){
    // Creates a new talon rotary motor
    rotaryMotor = new Talon(Constants.ROTARY_CHANNEL); 
  }

  public void rotate(){
    // We need to put a speed value
    System.out.println("Speed Setting.........");
    rotaryMotor.set(Constants.ROTARY_SPEED);
    System.out.println("Speed Set!!!!!!!!");
  }

    
  @Override
  public void periodic(){
    // This method will be called once per scheduler run
  }
}
