// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class MotorPorts {
        public static final int MOTOR_DRIVE_LEFT_ONE = 1; // SparkMax, NEO
        public static final int MOTOR_DRIVE_LEFT_TWO = 2; // SparkMax, NEO
        public static final int MOTOR_DRIVE_RIGHT_ONE = 3; // SparkMax, NEO
        public static final int MOTOR_DRIVE_RIGHT_TWO = 4; // SparkMax, NEO
    }

    public static final class PID {
        public static final double LEFT_DRIVE_PID0_P = 9e-5;
        public static final double LEFT_DRIVE_PID0_I = 0;
        public static final double LEFT_DRIVE_PID0_D = 0;
        public static final double LEFT_DRIVE_PID0_F = 0;

        public static final double RIGHT_DRIVE_PID0_P = 9e-5;
        public static final double RIGHT_DRIVE_PID0_I = 0;
        public static final double RIGHT_DRIVE_PID0_D = 0;
        public static final double RIGHT_DRIVE_PID0_F = 0;

        public static final double LEFT_DRIVE_PID1_P = 3e-1;
        public static final double LEFT_DRIVE_PID1_I = 0;
        public static final double LEFT_DRIVE_PID1_D = 0;
        public static final double LEFT_DRIVE_PID1_F = 1.5e-1;

        public static final double RIGHT_DRIVE_PID1_P = 3e-1;
        public static final double RIGHT_DRIVE_PID1_I = 0;
        public static final double RIGHT_DRIVE_PID1_D = 0;
        public static final double RIGHT_DRIVE_PID1_F = 1.5e-1;

        public static final double OUTPUT_RANGE_MIN = -1;
        public static final double OUTPUT_RANGE_MAX = 1;
    }

    public static final class GearRatios {
        public static final double GEAR_RATIO_DRIVE = 11.0 / 1.0;
    }

    public static final class CurrentLimit {
        public static final int CURRENT_LIMIT_DRIVE_LEFT = 40;
        public static final int CURRENT_LIMIT_DRIVE_RIGHT = 40;
    }

    public static final class ControllerPort {
        public static final int DRIVER_CONTROLLER = 1;
        public static final double TRIGGER_THRESHOLD = 0.1;
        public static final double JOYSTICK_THRESHOLD = 0.1;
    }

    public static final class AutoConstants {
        public static final double S_VOLTS = 0.209;
        public static final double V_VOLTS = 2.5;
        public static final double A_VOLTS = 0.55;

        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(25.6); // experimentally determined, inches
                                                                                    // to meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
        public static final double MAX_ACTUAL_SPEED_METERS_PER_SECOND = 4;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

        public static final double DELTA_V_THRESHOLD = 1.9; // Metres per second per loop cycle



    }

    public static final class Other {
        public static final double DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(6.0); // Meters
        public static final boolean GYRO_REVERSED = true;
        public static double INCHES_PER_REV = 1.7164; //Experimentally obtained Math.PI * DRIVE_WHEEL_DIAMETER / GEAR_RATIO_DRIVE;
        public static final int SECONDS_PER_MINUTE = 60;
    }
}
