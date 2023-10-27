// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;



public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private Relay m_relay;
  // Timer m_timer;
  // Solenoid m_solenoid;

  public ShooterSubsystem() {
    // m_solenoid = new Solenoid(null, 0);
    m_relay = new Relay(Constants.RELAY_CHANNEL,
                        Relay.Direction.kBoth);
    // m_timer = new Timer();
  }

  public void shoot() {
      // m_relay.set(Value.kForward);
      System.out.println("Relay On");
      // m_timer.delay(5);
      // m_relay.set(Value.kReverse);
      // System.out.println("Relay Off");
      m_relay.set(Value.kOn);

  }

  public void stop() {
    m_relay.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
