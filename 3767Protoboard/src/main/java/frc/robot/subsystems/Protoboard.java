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

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class Protoboard extends SubsystemBase {
  /** Initializes the Protoboard. */
  private WPI_TalonFX testingFalcon, leftFront, leftRear, rightFront, rightRear;
  private CANSparkMax testingNeo;
  private RelativeEncoder neoEncoder;
  private MotorBuilder motorBuilder = new MotorBuilder();
  private DifferentialDrive differentialDrive;

  private PhotonCamera camera  = new PhotonCamera("3767camera");

  public Protoboard() {
    //testing motors
    testingFalcon = motorBuilder.createFalcon(CAN.testingFalcon.ID, null, NeutralMode.Brake, false);
    testingNeo = motorBuilder.createNeo(CAN.testingNeo.ID, null, IdleMode.kBrake, false);
    neoEncoder = testingNeo.getEncoder();

    //left gearbox
    leftFront = motorBuilder.createFalcon(CAN.leftFront.ID, null, NeutralMode.Brake, false);
    leftRear = motorBuilder.createFalcon(CAN.leftRear.ID, leftFront, NeutralMode.Brake, false);

    //right gearbox
    rightFront = motorBuilder.createFalcon(CAN.rightFront.ID, null, NeutralMode.Brake, true);
    rightRear = motorBuilder.createFalcon(CAN.rightRear.ID, rightFront, NeutralMode.Brake, true);


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

  //camera methods
  public PhotonPipelineResult getCameraResult() {
    return camera.getLatestResult();
  }
}
