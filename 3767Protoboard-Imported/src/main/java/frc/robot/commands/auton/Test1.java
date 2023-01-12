package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Protoboard;

public class Test1 extends SequentialCommandGroup {
  /** Creates a new Test1. */
  public Test1(Protoboard protoboard) {
    addCommands();
  }
}
