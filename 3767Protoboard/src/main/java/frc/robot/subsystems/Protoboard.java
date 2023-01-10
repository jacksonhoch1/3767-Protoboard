package frc.robot.subsystems;

//wpilib utilities
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IDMap.CAN;
import frc.robot.utils.MotorBuilder;

//vendor libraries
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Protoboard extends SubsystemBase {
  /** Initializes the Protoboard. */
  private WPI_TalonFX testingFalcon, leftFront, leftRear, rightFront, rightRear;
  private CANSparkMax testingNeo;
  private RelativeEncoder neoEncoder;
  private MotorBuilder motorBuilder = new MotorBuilder();
  private DifferentialDrive differentialDrive;

  public Protoboard() {
    testingFalcon = motorBuilder.createFalcon(CAN.testingFalcon.ID, null, NeutralMode.Brake, false);

    //testing NEO
    testingNeo = new CANSparkMax(CAN_IDS.Protoboard.testingNeo, MotorType.kBrushless);
    testingNeo.restoreFactoryDefaults();
    testingNeo.setInverted(true);
    testingNeo.setIdleMode(IdleMode.kBrake);
    neoEncoder = testingNeo.getEncoder();

    //left gearbox
    //left front motor config
    leftFront = new WPI_TalonFX(CAN_IDS.Chassis.leftFront);
    leftFront.configFactoryDefault();
    leftFront.configAllSettings(configs);
    leftFront.setInverted(TalonFXInvertType.Clockwise);
    leftFront.setNeutralMode(NeutralMode.Brake);

    //left rear motor config
    leftRear = new WPI_TalonFX(CAN_IDS.Chassis.leftRear);
    leftRear.configFactoryDefault();
    leftRear.configAllSettings(configs);
    leftRear.follow(leftFront);
    leftRear.setInverted(TalonFXInvertType.FollowMaster);
    leftRear.setNeutralMode(NeutralMode.Brake);

    //right gearbox
    //right front motor config
    rightFront = new WPI_TalonFX(CAN_IDS.Chassis.rightFront);
    rightFront.configFactoryDefault();
    rightFront.configAllSettings(configs);
    rightFront.setInverted(TalonFXInvertType.CounterClockwise);
    rightFront.setNeutralMode(NeutralMode.Brake);

    //right rear motor config
    rightRear = new WPI_TalonFX(CAN_IDS.Chassis.rightRear);
    rightRear.configFactoryDefault();
    rightRear.configAllSettings(configs);
    rightRear.follow(rightFront);
    rightRear.setInverted(TalonFXInvertType.FollowMaster);
    rightRear.setNeutralMode(NeutralMode.Brake);

    this.differentialDrive = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Drive methods
  public void arcadeDrive(double xAxisThrottle, double zAxisRotation) {
    this.differentialDrive.arcadeDrive(-xAxisThrottle, zAxisRotation, true);
  }

  public void tankDrive(double leftThrottle, double rightThrottle) {
    this.differentialDrive.tankDrive(leftThrottle, rightThrottle);
  }

  //testing motor functions
  public void setFalcon(double speed) {
    testingFalcon.set(speed);
  }

  public void setNeo(double speed) {
    testingNeo.set(speed);
  }

  public double getNeoPos() {
    return neoEncoder.getPosition();
  }
}
