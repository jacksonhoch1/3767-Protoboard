package frc.robot.commands.auton;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;
import frc.robot.utils.Dashboard;

public class AimAtTarget extends CommandBase {
  /** Initializes a new AimAtTarget command. */
  final Protoboard protoboard;
  final double kP = 0.15;
  final double kD = 0.0;
  PIDController turnController = new PIDController(kP, 0.0, kD);
  PhotonPipelineResult result;
  double turningSpeed;
  public AimAtTarget(Protoboard protoboard) {
    this.protoboard = protoboard;
    addRequirements(protoboard);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putData(turnController);
    result = protoboard.getCameraResult();
    if (result.hasTargets()) {
      turningSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
      //protoboard.arcadeDrive(0.0, turningSpeed * 0.2);
      Dashboard.HAS_TARGET.put(true);
      Dashboard.TURN_TARGET.put(turningSpeed);
    } else {
      protoboard.arcadeDrive(0.0, 0.0);
      Dashboard.HAS_TARGET.put(false);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
