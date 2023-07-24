// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {

    /**********************************************************************/
    /***** CONSTANTS *****/

    private static final double WheelRadius = Units.inchesToMeters(2.0);
    private static final int EncoderResolution = 4096;

    private static final double ModuleMaxAngularVelocity = Drive.MaxAngularSpeed;
    private static final double ModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private static final double DriveKP = 1; // EHP change
    private static final double DriveKI = 0;
    private static final double DriveKD = 0;

    private static final double TurnKP = 1; // EHP change
    private static final double TurnKI = 0;
    private static final double TurnKD = 0;

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (2 * WheelRadius * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;

    /**********************************************************************/
    /**********************************************************************/

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final WPI_CANCoder driveEncoder;
    private final WPI_CANCoder turnEncoder;

    private final PIDController drivePIDController = new PIDController(DriveKP, DriveKI, DriveKD);
    private final ProfiledPIDController turnPIDController = // EHP change
            new ProfiledPIDController(
                    TurnKP,
                    TurnKI,
                    TurnKD,
                    new TrapezoidProfile.Constraints(ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration));


    public SwerveModule(
            int driveMotorID,
            int turningMotorID,
            int driveEncoderID,
            int turnEncoderID) {

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveEncoder = new WPI_CANCoder(driveEncoderID, "rio");
        turnEncoder = new WPI_CANCoder(turnEncoderID, "rio");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition())); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getPosition()));

        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        final double turnOutput = turnPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(0);
    }
}
