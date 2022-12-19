package frc.robot;

//wpilib
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//commands
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.SetTestingMotors;

//subsystems
import frc.robot.subsystems.Protoboard;

/**The main robot class that contains all of the configuration for the robot */
public class RobotContainer {
  //define the robots global objects, such as subsystems and controllers
  private final Protoboard m_protoboard;
  private final Joystick m_joystick;
  

  /** The main constructor for the robot. initializes objects like subsystems and controllers, and binds creates button bindings */
  public RobotContainer() {
    // Configure the button bindings
    m_protoboard = new Protoboard();
    m_joystick = new Joystick(0);

    configureButtonBindings();
  }

/**binds commands to buttons (only runs once, on robot startup) */
private void configureButtonBindings() {
    //set the default command to arcadeDrive, so it runs constantly
    m_protoboard.setDefaultCommand(getArcadeDriveCommand());

    //button creation
    JoystickButton enableTestingMotors = new JoystickButton(m_joystick, 2);

    //button execution
    enableTestingMotors.whileHeld(new SetTestingMotors(m_protoboard, () -> (m_joystick.getRawAxis(3) + 1) / 2, () -> (m_joystick.getRawAxis(4) + 1) / 2));

  }

  /**@return The command or command group that will run in autonomous mode */
  public Command getAutonomousCommand() {
    //nothing will run in autonomous
    return new InstantCommand();
  }

  /**@return The arcade drive command*/
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(m_protoboard, () -> m_joystick.getRawAxis(1), () -> m_joystick.getRawAxis(2));
  }
}
