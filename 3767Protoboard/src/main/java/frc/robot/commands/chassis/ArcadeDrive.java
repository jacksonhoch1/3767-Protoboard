package frc.robot.commands.chassis;

//utils
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

//subsystems
import frc.robot.subsystems.Protoboard;

public class ArcadeDrive extends CommandBase {
  private final Supplier<Double> throttle, turn;
  private final Protoboard protoboard;
  /** Initializes the ArcadeDrive command  */
  public ArcadeDrive(Protoboard protoboard, Supplier<Double> throttle, Supplier<Double> turn) {
    this.protoboard = protoboard;
    this.throttle = throttle;
    this.turn = turn;
    addRequirements(protoboard);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    protoboard.arcadeDrive(throttle.get(), turn.get());
  }

  @Override
  public void end(boolean interrupted) {
    protoboard.arcadeDrive(0, 0);
  }

  //end condition
  @Override
  public boolean isFinished() {
    return false;
  }
}
