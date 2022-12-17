// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Protoboard;

public class SetGearbox extends CommandBase {
  private Protoboard m_protoboard;
  private Supplier<Double> m_speed;
  //private double m_speed;
  /** Creates a new SetGearbox. */
  public SetGearbox(Protoboard protoboard, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_protoboard = protoboard;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_protoboard.setFalcons(m_speed.get());
    //m_protoboard.setNeos(m_speed2.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
