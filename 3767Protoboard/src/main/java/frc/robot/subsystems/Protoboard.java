// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CAN_IDS;

public class Protoboard extends SubsystemBase {
  /** Creates a new Protoboard. */
  private final WPI_TalonFX m_testingFalcon;
  private final CANSparkMax m_testingNeo;

  public Protoboard() {
    m_testingFalcon = new WPI_TalonFX(CAN_IDS.Protoboard.testingFalcon);
    m_testingFalcon.configFactoryDefault();
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_testingFalcon.configAllSettings(configs);
    m_testingFalcon.setInverted(TalonFXInvertType.Clockwise);
    m_testingFalcon.setNeutralMode(NeutralMode.Brake);

    m_testingNeo = new CANSparkMax(CAN_IDS.Protoboard.testingNeo, MotorType.kBrushless);
    m_testingNeo.restoreFactoryDefaults();
    m_testingNeo.setInverted(false);
    m_testingNeo.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
