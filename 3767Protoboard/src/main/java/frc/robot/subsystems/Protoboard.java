// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CAN_IDS;

public class Protoboard extends SubsystemBase {
  /** Creates a new Protoboard. */
  public final WPI_TalonFX m_testingFalcon, m_rightFrontGearbox, m_rightRearGearbox;
  public final CANSparkMax m_testingNeo;

  public Protoboard() {
    //configure motors
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    //testing falcon
    m_testingFalcon = new WPI_TalonFX(CAN_IDS.Protoboard.testingFalcon);
    m_testingFalcon.configFactoryDefault();
    m_testingFalcon.configAllSettings(configs);
    m_testingFalcon.setInverted(TalonFXInvertType.Clockwise);
    m_testingFalcon.setNeutralMode(NeutralMode.Brake);

    //testing NEO
    m_testingNeo = new CANSparkMax(CAN_IDS.Protoboard.testingNeo, MotorType.kBrushless);
    m_testingNeo.restoreFactoryDefaults();
    m_testingNeo.setInverted(false);
    m_testingNeo.setIdleMode(IdleMode.kBrake);

    //right gearbox
    //front motor config
    m_rightFrontGearbox = new WPI_TalonFX(CAN_IDS.Chassis.rightFrontGearbox);
    m_rightFrontGearbox.configFactoryDefault();
    m_rightFrontGearbox.configAllSettings(configs);
    m_rightFrontGearbox.setInverted(TalonFXInvertType.Clockwise);
    m_rightFrontGearbox.setNeutralMode(NeutralMode.Brake);

    //rear motor config
    m_rightRearGearbox = new WPI_TalonFX(CAN_IDS.Chassis.rightRearGearbox);
    m_rightRearGearbox.configFactoryDefault();
    m_rightRearGearbox.configAllSettings(configs);
    m_rightRearGearbox.setInverted(TalonFXInvertType.Clockwise);
    m_rightRearGearbox.setNeutralMode(NeutralMode.Brake);

  



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFalcons(double speed) {
    m_rightFrontGearbox.set(speed);
    m_rightRearGearbox.set(speed);
    
  }

  
}
