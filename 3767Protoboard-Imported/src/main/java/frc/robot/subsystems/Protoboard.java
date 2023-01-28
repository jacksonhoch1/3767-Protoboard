package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//wpilib utilities
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IDMap.CAN;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.MotorBuilder;

//vendor libraries
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;

import java.util.Map;

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
  private DoubleSolenoid primaryDoubleSolenoid;

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

    
    primaryDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 7);
    primaryDoubleSolenoid.set(Value.kReverse);


    this.differentialDrive = new DifferentialDrive(leftFront, rightFront);
    gyro.reset();
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
    differentialDrive.arcadeDrive(-xAxisThrottle, zAxisRotation, true);
  }

  public void tankDrive(double leftThrottle, double rightThrottle) {
    differentialDrive.tankDrive(leftThrottle, rightThrottle);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(rightVolts);
    rightFront.setVoltage(leftVolts);
    differentialDrive.feed();
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

  public void resetOdometry(Pose2d pose) {
    resetAll();
    odometry.resetPosition(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }


  public Command getRamseteCommand(String trajName, boolean resetPose) {
    Map<String, Trajectory> paths = RobotContainer.paths;
    Trajectory traj = RobotContainer.paths.get(trajName);
    SequentialCommandGroup commands = new SequentialCommandGroup();
    if (resetPose) commands.addCommands(new InstantCommand(() -> resetOdometry(traj.getInitialPose())));
    commands.addCommands(new InstantCommand(() -> field.getObject("traj").setTrajectory(traj)));

    commands.addCommands(new RamseteCommand(
      traj,
      this::getPose2d,
      new RamseteController(),
      new SimpleMotorFeedforward(
        Constants.Chassis.kS,
        Constants.Chassis.kV,
        Constants.Chassis.kA
        ),
        Constants.driveKinematics,
        this::getWheelSpeeds,
        new PIDController(Constants.Chassis.kP, 0.0, 0.0),
        new PIDController(Constants.Chassis.kP, 0.0, 0.0),
        this::tankDriveVolts,
        this
    ));
    return commands;
  }

  //Pneumatics methods
  public void setPrimarySolenoid(Value value) {
    primaryDoubleSolenoid.set(value);
  }

  public void extendPrimarySolenoid() {
    primaryDoubleSolenoid.set(Value.kForward);
  }

  public void retractPrimarySolenoid() {
    primaryDoubleSolenoid.set(Value.kReverse);
  }

  public void disablePrimarySolenoid() {
    primaryDoubleSolenoid.set(Value.kOff);
  }
  
}
