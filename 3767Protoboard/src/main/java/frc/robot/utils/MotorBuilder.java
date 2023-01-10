package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class MotorBuilder {
    public MotorBuilder() {}

    /**
     * Creates a new TalonFX motor controller (Falcon500). This was created to aid with creation of motors across multiple subsystems.
     * @param id The CAN Id of the motor controller.
     * @param follow The motor to follow. Supports any CTRE motor controller. If none, specify null.
     * @param neutralMode coast or brake when unpowered.
     * @param inverted Invert or not. Default is clockwise.
     * @return The fully configured motor
     */
    public WPI_TalonFX createFalcon(int id, BaseMotorController follow, NeutralMode neutralMode, boolean inverted) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        WPI_TalonFX motor = new WPI_TalonFX(id);
        motor.configFactoryDefault();
        motor.configAllSettings(configs);
        motor.setNeutralMode(neutralMode);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
        if (follow != null) {
            motor.follow(follow);
            motor.setInverted(TalonFXInvertType.FollowMaster);
        } else if (inverted) {
            motor.setInverted(TalonFXInvertType.CounterClockwise);
        } else {
            motor.setInverted(TalonFXInvertType.Clockwise);
        }
        return motor;
    }

    //TODO: NEO support
}