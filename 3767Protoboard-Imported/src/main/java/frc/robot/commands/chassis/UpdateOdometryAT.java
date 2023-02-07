package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class UpdateOdometryAT extends CommandBase{
    private final Protoboard protoboard;

    public UpdateOdometryAT(Protoboard protoboard) {
        this.protoboard = protoboard;
    }

    @Override
    public void initialize() {
        var result = protoboard.getCameraResult();
        var target = result.getBestTarget();

        if(result.hasTargets()) {
            protoboard.setOdometry(
                protoboard.fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d().plus(
                    new Transform2d(
                        target.getBestCameraToTarget().getTranslation().toTranslation2d(),
                        target.getBestCameraToTarget().getRotation().toRotation2d()
                    )
                )
            );
        }

        
    }

    @Override
    public boolean isFinished() {return true;}
    
}
