// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public final class Constants {
    public static final int LEFT_JOYSTICK_CHANNEL = 0;
    public static final int RIGHT_JOYSTICK_CHANNEL = 1;
    
    public static final int ROTARY_CHANNEL = 1; // Channel for rotating motor

    public static final double ROTARY_SPEED = 0.25; // Speed of the rotation 

    


    Encoder rotationEncoder;
    AnalogEncoder liftEncoder;
    PIDController liftPID;
    PIDController rotationPID;









}
