package frc.robot.utils;

//falcon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//neo
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorBuilder {
    public MotorBuilder() {}

    /**
     * Creates a new TalonFX motor controller (Falcon500). This was created to aid with creation of motors across multiple subsystems.
     * @param id The CAN Id of the motor controller.
     * @param leader The motor to follow. Supports any CTRE motor controller. If none, specify null.
     * @param neutralMode coast or brake when unpowered.
     * @param inverted Invert or not. Default is clockwise.
     * If this motor is a follower default is followMaster, but if true, opposeMaster. 
     * @return The fully configured motor
     */
    public WPI_TalonFX createFalcon(int id, BaseMotorController leader, NeutralMode neutralMode, boolean isInverted) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        WPI_TalonFX motor = new WPI_TalonFX(id);
        motor.configFactoryDefault();
        motor.configAllSettings(configs);
        motor.setNeutralMode(neutralMode);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
        if (leader != null) {
            motor.follow(leader);
            if (isInverted) {
                motor.setInverted(TalonFXInvertType.OpposeMaster);
            } else {
                motor.setInverted(TalonFXInvertType.FollowMaster);;
            }
        } else {
            motor.setInverted(isInverted);
        }
        return motor;
    }

    public CANSparkMax createNeo(int id, CANSparkMax leader, IdleMode idleMode, boolean isInverted) {
        CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(idleMode);
        if (leader != null) {
            motor.follow(leader);
        } else if (isInverted) {
            motor.setInverted(isInverted);
        }
        return motor; 
    }
}