// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class TankDrive extends CommandBase {
  Protoboard protoboard;
  Supplier<Double> leftThrottle, rightThrottle;

  /** Creates a new TankDrive. */
  public TankDrive(Protoboard protoboard, Supplier<Double> leftThrottle, Supplier<Double> rightThrottle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.protoboard = protoboard;
    this.leftThrottle = leftThrottle;
    this.rightThrottle = rightThrottle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    protoboard.tankDrive(leftThrottle.get(), rightThrottle.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    protoboard.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
