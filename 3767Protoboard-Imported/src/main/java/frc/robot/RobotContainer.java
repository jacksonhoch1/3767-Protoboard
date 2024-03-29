package frc.robot;

//utils
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.utils.AutonomousLoader;
import frc.robot.utils.Dashboard;
//wpilib
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetSolenoid;
//commands
import frc.robot.commands.SetTestingMotors;
import frc.robot.commands.auton.AimAtTarget;
import frc.robot.commands.auton.FollowTarget;
import frc.robot.commands.auton.ResetOdometry;
import frc.robot.commands.auton.Test1;
import frc.robot.commands.auton.getInRangeOfTarget;
import frc.robot.commands.chassis.ArcadeDrive;
import frc.robot.commands.chassis.TankDrive;
import frc.robot.commands.chassis.UpdateOdometryAT;
//subsystems
import frc.robot.subsystems.Protoboard;

/**The main robot class that contains all of the configuration for the robot */
public class RobotContainer {
  //define the robot's global objects, such as subsystems and controllers
  private final Protoboard m_protoboard;
  public final Joystick m_joystick;

  private final PathConstraints pathConstraints = new PathConstraints(15, 15);
  public static Map<String, Trajectory> paths;
  private AutonomousLoader autoLoader;
  

  /** The main constructor for the robot. initializes objects like subsystems and controllers, and creates button bindings */
  public RobotContainer() {
    // Configure the button bindings

    paths = loadPaths(List.of(
      "Test1"
    ));

    m_protoboard = new Protoboard(this);
    m_joystick = new Joystick(0);

    autoLoader = new AutonomousLoader(m_protoboard, paths);

    //SmartDashboard.putData("autoChooser", autoLoader.getSendableChooser());

    configureButtonBindings();
  }

/**binds commands to buttons (only runs once, on robot startup) */
private void configureButtonBindings() {
    //set the default command to arcadeDrive, so it runs constantly
    m_protoboard.setDefaultCommand(getArcadeDriveCommand());

    //button creation
    JoystickButton getInRangeOfTarget = new JoystickButton(m_joystick, 1);
    JoystickButton aimAtTarget = new JoystickButton(m_joystick, 3);
    JoystickButton followTarget = new JoystickButton(m_joystick, 2);
    JoystickButton updateOdometry = new JoystickButton(m_joystick, 5);

    JoystickButton enableTestingMotors = new JoystickButton(m_joystick, 4);
    POVButton resetOdometry = new POVButton(m_joystick, 0);
    POVButton extendSolenoid = new POVButton(m_joystick, 90);
    POVButton retractSolenoid = new POVButton(m_joystick, 270);
    

    //button execution
    enableTestingMotors.whileTrue(new SetTestingMotors(m_protoboard, 
      () -> (m_joystick.getRawAxis(3) + 1) / 2,     //the arithmetic in these lines changes the trigger output 
      () -> (m_joystick.getRawAxis(4) + 1) / 2));   //to be 0.0 to 1.0, instead of -1.0 to 1.0
    aimAtTarget.whileTrue(new AimAtTarget(m_protoboard));
    getInRangeOfTarget.whileTrue(new getInRangeOfTarget(m_protoboard));
    followTarget.whileTrue(new FollowTarget(m_protoboard, 1));

    resetOdometry.onTrue(new InstantCommand(() -> m_protoboard.setOdometry(new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0)))));
    updateOdometry.onTrue(new UpdateOdometryAT(m_protoboard));

    extendSolenoid.onTrue(new SetSolenoid(m_protoboard, Value.kForward));
    retractSolenoid.onTrue(new SetSolenoid(m_protoboard, Value.kReverse));

  }

  /**@return The command or command group that will run in autonomous mode */
  public Command getAutonomousCommand() {
    //nothing will run in autonomous
    //return new InstantCommand();
    //return autoLoader.getCurrentSelection();
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