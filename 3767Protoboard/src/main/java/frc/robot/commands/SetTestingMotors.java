// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class SetTestingMotors extends CommandBase {
  Protoboard protoboard;
  Supplier<Double> falcon, neo;
  /** Creates a new SetTestingMotors. */
  public SetTestingMotors(Protoboard protoboard, Supplier<Double> falcon, Supplier<Double> neo) {
    // Use addRequirements() here to declare subsystem dependencies.
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
