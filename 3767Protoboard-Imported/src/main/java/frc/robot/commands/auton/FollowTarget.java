package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;
import frc.robot.utils.Dashboard;

public class FollowTarget extends CommandBase{
    private final Protoboard protoboard;
    private PIDController turnController = new PIDController(0.025, 0, 0);
    private PIDController speedController = new PIDController(0.5, 0, 0);

    double turnSpeed, forwardSpeed;
    double targetDistance;

    public FollowTarget(Protoboard protoboard, double targetDistance) {
        this.protoboard = protoboard;
        this.targetDistance = targetDistance;
        addRequirements(protoboard);

        turnController.setTolerance(5);
        // speedController.setTolerance(1);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var result = protoboard.getCameraResult();

        if (result.hasTargets()) {
            Dashboard.HAS_TARGET.put(true);
            var target = result.getBestTarget();
            double range = target.getBestCameraToTarget().getX();
            double yaw = target.getYaw();

            if(!turnController.atSetpoint()) {
                turnSpeed = turnController.calculate(yaw, 0);
                Dashboard.TURN_TARGET.put(turnSpeed);
                protoboard.arcadeDrive(0, turnSpeed);
            } else if (!speedController.atSetpoint()) {
                forwardSpeed = speedController.calculate(range, targetDistance) * 4;
                Dashboard.SPEED_TARGET.put(forwardSpeed);
                protoboard.arcadeDrive(forwardSpeed, 0);
            }

            

            

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