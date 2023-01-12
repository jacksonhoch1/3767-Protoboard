package frc.robot.commands.chassis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class TankDrive extends CommandBase {
  Protoboard protoboard;
  Supplier<Double> leftThrottle, rightThrottle;

  /** Initializes a new TankDrive command */
  public TankDrive(Protoboard protoboard, Supplier<Double> leftThrottle, Supplier<Double> rightThrottle) {
    this.protoboard = protoboard;
    this.leftThrottle = leftThrottle;
    this.rightThrottle = rightThrottle;
    addRequirements(protoboard);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    protoboard.tankDrive(leftThrottle.get(), rightThrottle.get());
  }

  @Override
  public void end(boolean interrupted) {
    protoboard.tankDrive(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
