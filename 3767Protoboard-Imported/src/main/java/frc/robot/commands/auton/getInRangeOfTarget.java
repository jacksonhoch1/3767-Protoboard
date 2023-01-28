package frc.robot.commands.auton;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Protoboard;

public class getInRangeOfTarget extends CommandBase {
  Protoboard protoboard;
  PIDController controller = new PIDController(0.5, 0, 0);

  public getInRangeOfTarget(Protoboard protoboard) {
    this.protoboard = protoboard;
    addRequirements(protoboard);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double forwardSpeed;
    var result = protoboard.getCameraResult();

    if (result.hasTargets()) {
      double range = result.getBestTarget().getBestCameraToTarget().getX();
      // double range = PhotonUtils.calculateDistanceToTargetMeters(
      //   Constants.Camera.CAMERA_HEIGHT,
      //   Constants.Camera.TARGET_HEIGHT,
      //   Constants.Camera.CAMERA_PITCH_RADIANS,
      //   Units.degreesToRadians(result.getBestTarget().getPitch()));
      
      forwardSpeed = controller.calculate(range, 1) * 4;
      SmartDashboard.putNumber("speed", forwardSpeed);
    } else {forwardSpeed = 0;}

    protoboard.arcadeDrive(forwardSpeed, 0);
  }
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return false;}
}