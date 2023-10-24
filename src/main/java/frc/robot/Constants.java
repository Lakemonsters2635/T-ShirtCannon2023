// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    // public static final int LEFT_JOYSTICK_CHANNEL = 3; // For xbox controller
    // public static final int RIGHT_JOYSTICK_CHANNEL = 3; // For xbox controller
    public static final int CONTROLLER_CHANNEL = 3; // For xbox controller
    public static final int RELAY_CHANNEL = 0; // Channel for relay (controls the shooting)
    public static final int ROTARY_CHANNEL = 1; // Channel for rotating motor

    public static final double ROTARY_SPEED = 0.25; // Speed of the rotation 

    // Motor Constants
    // TODO update these to the proper id
    // TODO choose what constant names you want... id_* is legacy nomenclature
    public static final int id_LEFTMOTOR1 = 8;
    public static final int id_LEFTMOTOR2 = 9;

    public static final int id_RIGHTMOTOR1 = 1;
    public static final int id_RIGHTMOTOR2 = 2;
}
