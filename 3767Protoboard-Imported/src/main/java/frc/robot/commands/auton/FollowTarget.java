package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;
import frc.robot.utils.Dashboard;

public class FollowTarget extends CommandBase{
    private final Protoboard protoboard;
    private PIDController turnController = new PIDController(0.1, 0, 0.1);
    private PIDController speedController = new PIDController(0.5, 0, 0);

    double turnSpeed, forwardSpeed;
    double targetDistance;

    public FollowTarget(Protoboard protoboard, double targetDistance) {
        this.protoboard = protoboard;
        this.targetDistance = targetDistance;
        addRequirements(protoboard);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var result = protoboard.getCameraResult();

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            double range = target.getBestCameraToTarget().getX();

            forwardSpeed = speedController.calculate(range, targetDistance) * 4;
            turnSpeed = turnController.calculate(target.getYaw(), 0);

            protoboard.arcadeDrive(forwardSpeed, turnSpeed * 0.2);

            Dashboard.HAS_TARGET.put(true);
            Dashboard.SPEED_TARGET.put(forwardSpeed);
            Dashboard.TURN_TARGET.put(turnSpeed);
        } else {
            protoboard.arcadeDrive(0, 0);
            Dashboard.HAS_TARGET.put(false);
        }
    }

    @Override
    public void end(boolean isInterrupted) {}

    @Override
    public boolean isFinished() {return false;}
}