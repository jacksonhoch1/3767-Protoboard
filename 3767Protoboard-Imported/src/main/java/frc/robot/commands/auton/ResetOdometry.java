// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class ResetOdometry extends CommandBase {
  private Protoboard protoboard;
  /** Creates a new ResetOdometry. */
  public ResetOdometry(Protoboard protoboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.protoboard = protoboard;
    addRequirements(protoboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    protoboard.resetOdometry(protoboard.getPose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
