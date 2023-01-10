package frc.robot.commands.auton;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class AimAtTarget extends CommandBase {
  /** Initializes a new AimAtTarget command. */
  final Protoboard protoboard;
  final double kP = 0.1;
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
    result = protoboard.getCameraResult();
    if (result.hasTargets()) {
      turningSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
      protoboard.arcadeDrive(0.0, turningSpeed);
    } else {
      protoboard.arcadeDrive(0.0, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
