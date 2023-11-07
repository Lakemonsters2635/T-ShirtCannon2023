// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RotarySubsystem extends SubsystemBase{
  /** Creates a new RotarySubsystem. */
  public Talon rotaryMotor; // Instantiates rotating motor
  private Encoder rotationEncoder;
  private double theta;
  private double rotationEncoderCounts;

  public RotarySubsystem(){
    // Creates a new talon rotary motor
    rotationEncoder = new Encoder(Constants.ROTATION_ENCODER_1, Constants.ROTATION_ENCODER_2);
    rotaryMotor = new Talon(Constants.ROTARY_CHANNEL); 
  }

  public void rotate(){
    // We need to put a speed value
   // System.out.println("Speed Setting.........");
    rotaryMotor.set(Constants.ROTARY_SPEED);
    //System.out.println("Speed Set!!!!!!!!");
  }
  public void stop(){
    rotaryMotor.set(Constants.ROTARY_STOP);
  }

  public void resetEncoderCounts(){
    rotationEncoder.reset();
  }
  public double getEncoderCounts(){
    //System.out.println("Count is " + theta); 
    
    return rotationEncoder.getRaw();
  }
    
  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    
  }
}
