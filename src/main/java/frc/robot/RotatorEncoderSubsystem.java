// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotatorEncoderSubsystem extends SubsystemBase {
  /** Creates a new RotatorEncoderSubsystem. */
  Encoder rotationEncoder;
  public RotatorEncoderSubsystem() 
  {
    rotationEncoder = new Encoder(Constants.ROTATION_ENCODER_1, Constants.ROTATION_ENCODER_2);
    
  }
  
  public void getEncoderCounts(){
    double rotationEncoderCounts = rotationEncoder.get();
    System.out.println("Count is " + rotationEncoderCounts);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


   
  }
}
// test git