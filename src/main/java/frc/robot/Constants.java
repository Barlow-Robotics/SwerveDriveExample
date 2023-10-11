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

        // LeftFront = 1
        // rightFront = 2
        // leftBack = 3
        // rightBack = 4

        // Encoder = 1{locationOnBot}
        public static final int FrontLeftTurnEncoderID = 11;
        public static final int FrontRightTurnEncoderID = 12;
        public static final int BackLeftTurnEncoderID = 50;
        public static final int BackRightTurnEncoderID = 14;

        // BackMotorID = 2{locationOnBot} // Base
        public static final int FrontLeftDriveMotorID = 21;
        public static final int FrontRightDriveMotorID = 22;
        public static final int BackLeftDriveMotorID = 23;
        public static final int BackRightDriveMotorID = 24;

        // TurnMotorID = 3{locationOnBot} // Side
        public static final int FrontLeftTurnMotorID = 31;
        public static final int FrontRightTurnMotorID = 32;
        public static final int BackLeftTurnMotorID = 33;
        public static final int BackRightTurnMotorID = 34;

        /* DRIVE */
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
