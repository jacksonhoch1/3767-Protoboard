package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class SetSolenoid extends CommandBase {
  private final Protoboard protoboard;
  private Value value;

  /** Initializes a new SetSolenoid command. */
  public SetSolenoid(Protoboard protoboard, Value value) {
    this.protoboard = protoboard;
    this.value = value;
  }

  @Override
  public void initialize() {
    protoboard.setPrimarySolenoid(value);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
