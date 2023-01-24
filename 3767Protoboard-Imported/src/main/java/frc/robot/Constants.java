// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //10:60, 16:32
    // 6        2
    public static final double WHEEL_DIAMETER_INCHES = 4.25;
    public static final double ENCODER_COUNTS_PER_REV = 2048;
    public static final double GEAR_RATIO = 12;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_CIRCUMFERENCE_INCHES);
    public static final double COUNTS_PER_METER = (GEAR_RATIO * ENCODER_COUNTS_PER_REV) / WHEEL_CIRCUMFERENCE_METERS;
    public static final double TRACK_WIDTH = 0.65347;
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final class Chassis {
        public static final double kS = 0.18426;
        public static final double kV = 3.8389;
        public static final double kA = 0.23194;
        public static final double kP = 3.8503;
    }

}
