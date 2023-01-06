// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.ArcadeDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Protoboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class straight2s extends SequentialCommandGroup {
  private final Protoboard protoboard;
  /** Creates a new straight2s. */
  public straight2s(Protoboard protoboard) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.protoboard = protoboard;
    addCommands(
      new ArcadeDrive(protoboard, () -> 0.2, () -> 0.0),
      new WaitCommand(1),
      new ArcadeDrive(protoboard, () -> 0.0, () -> 0.0)

    );
  }
}
