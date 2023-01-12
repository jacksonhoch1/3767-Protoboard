package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//wpilib utilities
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IDMap.CAN;
import frc.robot.Constants;
import frc.robot.utils.MotorBuilder;

//vendor libraries
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class Protoboard extends SubsystemBase {
  /** Initializes the Protoboard. */
  private WPI_TalonFX testingFalcon, leftFront, leftRear, rightFront, rightRear;
  private CANSparkMax testingNeo;
  private MotorBuilder motorBuilder = new MotorBuilder();
  private DifferentialDrive differentialDrive;
  private RelativeEncoder neoEncoder;
  private AHRS gyro = new AHRS();
  private DifferentialDriveOdometry odometry;
  private Field2d field = new Field2d();

  private PhotonCamera camera  = new PhotonCamera("OV5647");

  public Protoboard() {
    //testing motors
    testingFalcon = motorBuilder.createFalcon(CAN.testingFalcon.ID, null, NeutralMode.Brake, false);
    testingNeo = motorBuilder.createNeo(CAN.testingNeo.ID, null, IdleMode.kBrake, false);
    neoEncoder = testingNeo.getEncoder();

    //left gearbox
    leftFront = motorBuilder.createFalcon(CAN.leftFront.ID, null, NeutralMode.Brake, true);
    leftRear = motorBuilder.createFalcon(CAN.leftRear.ID, leftFront, NeutralMode.Brake, false);


    //right gearbox
    rightFront = motorBuilder.createFalcon(CAN.rightFront.ID, null, NeutralMode.Brake, false);
    rightRear = motorBuilder.createFalcon(CAN.rightRear.ID, rightFront, NeutralMode.Brake, false);


    this.differentialDrive = new DifferentialDrive(leftFront, rightFront);

    gyro.calibrate();
    resetEncoders();

    odometry = new DifferentialDriveOdometry(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

    SmartDashboard.putData("field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
    field.setRobotPose(getPose2d());
  }

  //Drive methods
  public void arcadeDrive(double xAxisThrottle, double zAxisRotation) {
    this.differentialDrive.arcadeDrive(-xAxisThrottle, zAxisRotation, true);
  }

  public void tankDrive(double leftThrottle, double rightThrottle) {
    this.differentialDrive.tankDrive(leftThrottle, rightThrottle);
  }

  //encoder methods
  public double getLeftDistanceMeters() {
    return leftFront.getSelectedSensorPosition() / Constants.COUNTS_PER_METER;
  }

  public double getRightDistanceMeters() {
    return rightFront.getSelectedSensorPosition() / Constants.COUNTS_PER_METER;
  }

  public double getLeftVelocityMetersPerSecond() {
    return leftFront.getSelectedSensorVelocity() / Constants.COUNTS_PER_METER;
  }

  public double getRightVelocityMetersPerSecond() {
    return rightFront.getSelectedSensorVelocity() / Constants.COUNTS_PER_METER;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocityMetersPerSecond(),
      getRightVelocityMetersPerSecond()
    );
  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
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

  //gyro methods
  public void resetGyro() {
    gyro.reset();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public Rotation2d getGyroRotation2d() {
    return gyro.getRotation2d();
  }

  //odometry methods
  public void resetAll() {
    resetEncoders();
    resetGyro();
  }

  public void resetOdometry() {
    resetEncoders();
    odometry.resetPosition(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), null);
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }
}
