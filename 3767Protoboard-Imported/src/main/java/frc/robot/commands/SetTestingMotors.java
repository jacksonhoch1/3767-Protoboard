/**a command for controlling the testing motors on the protoboard */

package frc.robot.commands;

//wpi and java utilities
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

//subsystems
import frc.robot.subsystems.Protoboard;

public class SetTestingMotors extends CommandBase {
  Protoboard protoboard;
  Supplier<Double> falcon, neo;
  /** Initializes SetTestingMotors. */
  public SetTestingMotors(Protoboard protoboard, Supplier<Double> falcon, Supplier<Double> neo) {
    //sets the global variables to arguments
    this.protoboard = protoboard;
    this.falcon = falcon;
    this.neo = neo;
    addRequirements(protoboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    protoboard.setFalcon(falcon.get());
    protoboard.setNeo(neo.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    protoboard.setFalcon(0);
    protoboard.setNeo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
