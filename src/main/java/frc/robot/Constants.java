// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

    public static final double SecondsPerMinute = 60;

    public class CanIDs {
        /* DRIVE */
        public static final int FrontLeftDriveMotorID = 1; // EHP change
        public static final int FrontLeftTurnMotorID = 2; // EHP change
        public static final int FrontLeftTurnEncoderID = 2; // EHP change

        public static final int FrontRightDriveMotorID = 3; // EHP change
        public static final int FrontRightTurnMotorID = 4; // EHP change
        public static final int FrontRightTurnEncoderID = 6; // EHP change

        public static final int BackLeftDriveMotorID = 5; // EHP change
        public static final int BackLeftTurnMotorID = 6; // EHP change
        public static final int BackLeftTurnEncoderID = 10; // EHP change

        public static final int BackRightDriveMotorID = 7; // EHP change
        public static final int BackRightTurnMotorID = 8; // EHP change 
        public static final int BackRightTurnEncoderID = 14; // EHP change 
    }
    public class DriveConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5; // Might need to change
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // Might need to change
    }
    public class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4; // Might need to change
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10; // Might need to change
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // Might need to change
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    }
}
