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

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CAN_IDS;

public class Protoboard extends SubsystemBase {
  /** Creates a new Protoboard. */
  public WPI_TalonFX m_testingFalcon, m_leftFront, m_leftRear, m_rightFront, m_rightRear;
  public CANSparkMax m_testingNeo;
  public DifferentialDrive m_differentialDrive;

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
    m_testingNeo.setInverted(true);
    m_testingNeo.setIdleMode(IdleMode.kBrake);

    //left gearbox
    //left front motor config
    m_leftFront = new WPI_TalonFX(CAN_IDS.Chassis.leftFront);
    m_leftFront.configFactoryDefault();
    m_leftFront.configAllSettings(configs);
    m_leftFront.setInverted(TalonFXInvertType.Clockwise);
    m_leftFront.setNeutralMode(NeutralMode.Brake);

    //left rear motor config
    m_leftRear = new WPI_TalonFX(CAN_IDS.Chassis.leftRear);
    m_leftRear.configFactoryDefault();
    m_leftRear.configAllSettings(configs);
    m_leftRear.follow(m_leftFront);
    m_leftRear.setInverted(TalonFXInvertType.FollowMaster);
    m_leftRear.setNeutralMode(NeutralMode.Brake);

    //right gearbox
    //right front motor config
    m_rightFront = new WPI_TalonFX(CAN_IDS.Chassis.rightFront);
    m_rightFront.configFactoryDefault();
    m_rightFront.configAllSettings(configs);
    m_rightFront.setInverted(TalonFXInvertType.CounterClockwise);
    m_rightFront.setNeutralMode(NeutralMode.Brake);

    //right rear motor config
    m_rightRear = new WPI_TalonFX(CAN_IDS.Chassis.rightRear);
    m_rightRear.configFactoryDefault();
    m_rightRear.configAllSettings(configs);
    m_rightRear.follow(m_rightFront);
    m_rightRear.setInverted(TalonFXInvertType.FollowMaster);
    m_rightRear.setNeutralMode(NeutralMode.Brake);

    this.m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

  



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFalcon(double speed) {
    m_testingFalcon.set(speed);
    
  }

  public void setNeo(double speed) {
    m_testingNeo.set(speed);
  }

  public void arcadeDrive(double xAxisThrottle, double zAxisRotation) {
    this.m_differentialDrive.arcadeDrive(-xAxisThrottle, zAxisRotation, true);
  }
}
