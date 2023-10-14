
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

// initial code from T-Shirt-Robot ArcadeDrive
// https://github.com/2635Lakemonsters/T-Shirt-Robot/blob/master/src/edu/wpi/first/wpilibj/templates/ArcadeDrive.java
// public class ArcadeDrive
public class TankDriveSubsystem
{
    Talon lMotor1;
    Talon lMotor2;
    Talon rMotor1;
    Talon rMotor2;

    /**
     *
     */
    public TankDriveSubsystem(Talon leftMotor1, Talon leftMotor2, Talon rightMotor1, Talon rightMotor2)
    {
        //super(leftMotor1, leftMotor2, rightMotor1, rightMotor2);
        lMotor1 = leftMotor1;
        lMotor2 = leftMotor2;
        rMotor1 = rightMotor1;
        rMotor2 = rightMotor2;
        /**
         * If problems continue, uncomment line below.
         */
        //setSafetyEnabled(false);
    }

    public void drive(double lJoystick, double rJoystick)
    {
        lMotor1.set(lJoystick);
        lMotor2.set(lJoystick);
        rMotor1.set(rJoystick);
        rMotor2.set(rJoystick);
    }
}
