// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class ArcadeDrive extends CommandBase {
  private final Supplier<Double> m_throttle, m_turn;
  private final Protoboard m_protoboard;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Protoboard protoboard, Supplier<Double> xAxisThrottle, Supplier<Double> zAxisRotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_protoboard = protoboard;
    m_throttle = xAxisThrottle;
    m_turn = zAxisRotate;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_protoboard.arcadeDrive(m_throttle.get(), m_turn.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_protoboard.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
