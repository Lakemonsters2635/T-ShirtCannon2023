// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;


/** Add your docs here. */
public final class Constants {
    public final static int CONTROLLER_CHANNEL = 2;
    public final static int leftJoystick = 0;
    public final static int rightJoystick = 1;

    //xbox controller Constants
    public final int LEFT_X_AXIS = 0;
    public final int LEFT_Y_AXIS = 1;
    public final static int LEFT_TRIGGER = 1;
    public final static int RIGHT_TRIGGER = 3;
    public final int RIGHT_X_AXIS = 4;
    public final int RIGHT_Y_AXIS = 5;

    //Encoder Constants
    public final static int ROTATION_ENCODER_1 = 2; //DIO1
    public final static int ROTATION_ENCODER_2 = 1; //DIO2

    
    
    public static final int ROTARY_CHANNEL = 4; // Channel for rotating motor
    public static final double ROTARY_SPEED = 1; // Speed of the rotation 
    public static final double ROTARY_STOP = 0; //Stop rotation

    Encoder rotationEncoder;
    AnalogEncoder liftEncoder;
    PIDController liftPID;
    PIDController rotationPID;
    

        
}
