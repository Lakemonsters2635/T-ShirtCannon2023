package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmRotationCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DifferentialDrive2;
import frc.robot.subsystems.RotarySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    public final RotarySubsystem m_RotarySubsystem = new RotarySubsystem();
    public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public final DifferentialDrive2 m_tankDrive = new DifferentialDrive2(); 

    public final ArmRotationCommand m_ArmRotationCommand = new ArmRotationCommand(m_RotarySubsystem);
    public final ShooterCommand shooterCommand = new ShooterCommand(m_shooterSubsystem);

    public final Joystick m_leftStick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);
    public final Joystick m_rightStick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
    // private CommandXboxController m_controller;
    public final XboxController m_controller = new XboxController(Constants.CONTROLLER_CHANNEL);

    public RobotContainer() {

        configureBindings();
    }

    private void configureBindings(){
        
        //xbox controller buttons
        Trigger shooterButtonX = new JoystickButton(m_controller, XboxController.Button.kB.value);
        shooterButtonX.onTrue(new ShooterCommand(m_shooterSubsystem));

        Trigger rotationButtonX = new JoystickButton(m_controller, XboxController.Button.kA.value);
        rotationButtonX.onTrue(new ArmRotationCommand(m_RotarySubsystem));
        
        Trigger combinedButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
        combinedButton.onTrue(new SequentialCommandGroup(new ArmRotationCommand(m_RotarySubsystem),new ShooterCommand(m_shooterSubsystem)));
            
        //joystick buttons
        Trigger shooterButtonJ = new JoystickButton(m_rightStick, Constants.SHOOTER_BUTTON);
        shooterButtonJ.onTrue(new ShooterCommand(m_shooterSubsystem));

        Trigger rotationButtonJ = new JoystickButton(m_leftStick, Constants.ROTATION_BUTTON);
        rotationButtonJ.onTrue(new ArmRotationCommand(m_RotarySubsystem));

        System.out.println("Robot.configureBindigs()");
    }

    public Command getAutonomousCommand() {
        return null; 
    }
}
