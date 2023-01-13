package frc.robot;

//utils
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.utils.AutonomousLoader;

//wpilib
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//commands
import frc.robot.commands.SetTestingMotors;
import frc.robot.commands.auton.AimAtTarget;
import frc.robot.commands.auton.ResetOdometry;
import frc.robot.commands.chassis.ArcadeDrive;
import frc.robot.commands.chassis.TankDrive;

//subsystems
import frc.robot.subsystems.Protoboard;

/**The main robot class that contains all of the configuration for the robot */
public class RobotContainer {
  //define the robot's global objects, such as subsystems and controllers
  private final Protoboard m_protoboard;
  private final Joystick m_joystick;

  private final PathConstraints pathConstraints = new PathConstraints(15, 15);
  public static Map<String, Trajectory> paths;
  private AutonomousLoader autoLoader;
  

  /** The main constructor for the robot. initializes objects like subsystems and controllers, and creates button bindings */
  public RobotContainer() {
    // Configure the button bindings

    paths = loadPaths(List.of(
      "MobilityEngage"
    ));

    m_protoboard = new Protoboard();
    m_joystick = new Joystick(0);

    autoLoader = new AutonomousLoader(m_protoboard, paths);

    configureButtonBindings();
  }

/**binds commands to buttons (only runs once, on robot startup) */
private void configureButtonBindings() {
    //set the default command to arcadeDrive, so it runs constantly
    m_protoboard.setDefaultCommand(getArcadeDriveCommand());

    //button creation
    JoystickButton enableTestingMotors = new JoystickButton(m_joystick, 2);
    JoystickButton aimAtTarget = new JoystickButton(m_joystick, 3);
    POVButton resetOdometry = new POVButton(m_joystick, 0);

    //button execution
    enableTestingMotors.whileTrue(new SetTestingMotors(m_protoboard, 
      () -> (m_joystick.getRawAxis(3) + 1) / 2,     //the arithmetic in these lines changes the trigger output 
      () -> (m_joystick.getRawAxis(4) + 1) / 2));   //to be 0.0 to 1.0, instead of -1.0 to 1.0
    aimAtTarget.whileTrue(new AimAtTarget(m_protoboard));

    resetOdometry.onTrue(new ResetOdometry(m_protoboard));

  }

  /**@return The command or command group that will run in autonomous mode */
  public Command getAutonomousCommand() {
    //nothing will run in autonomous
    //return new InstantCommand();
    return new AimAtTarget(m_protoboard);
  }

  /**@return the arcade drive command*/
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(m_protoboard, () -> m_joystick.getRawAxis(1), () -> -m_joystick.getRawAxis(2));
  }

  /**@return the tank drive command */
  public Command getTankDriveCommand() {
    return new TankDrive(m_protoboard, () -> -m_joystick.getRawAxis(1), () -> -m_joystick.getRawAxis(5));
  }

  public Map<String, Trajectory> loadPaths(List<String> names) {
    Map<String, Trajectory> trajectories = new HashMap<>();

    for (String n : names) {
      trajectories.put(n, PathPlanner.loadPath(n, pathConstraints));
    }

    return trajectories;
  }
}