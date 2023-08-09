// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import org.littletonrobotics.junction.Logger;


public class Drive extends SubsystemBase {

    /*******************************************************************************/
    /***** CONSTANTS *****/

    public static final double MaxSpeed = 3.0; // meters per second
    public static final double MaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final boolean GyroReversed = false;

    /*******************************************************************************/
    /*******************************************************************************/

    private final SwerveModule frontLeft = new SwerveModule(
            CanIDs.FrontLeftDriveMotorID,
            CanIDs.FrontLeftTurnMotorID,
            CanIDs.FrontLeftTurnEncoderID);

    private final SwerveModule frontRight = new SwerveModule(
            CanIDs.FrontRightDriveMotorID,
            CanIDs.FrontRightTurnMotorID,
            CanIDs.FrontRightTurnEncoderID);

    private final SwerveModule backLeft = new SwerveModule(
            CanIDs.BackLeftDriveMotorID,
            CanIDs.BackLeftTurnMotorID,
            CanIDs.BackLeftTurnEncoderID);

    private final SwerveModule backRight = new SwerveModule(
            CanIDs.BackRightDriveMotorID,
            CanIDs.BackRightTurnMotorID,
            CanIDs.BackRightTurnEncoderID);

    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); // EHP change
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381); // EHP change
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381); // EHP change
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381); // EHP change

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });

    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[4] ;


    public Drive() {
    }

    @Override
    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        Logger.getInstance().recordOutput("Pose", odometry.getPoseMeters());

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        Logger.getInstance().recordOutput("SwerveStates/States", swerveModuleStates);

    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, MaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate() * (GyroReversed ? -1.0 : 1.0); // degrees per second
    }

    public void simulationInit() {
        frontLeft.simulationInit();
        frontRight.simulationInit();
        backLeft.simulationInit();
        backRight.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        var modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            } ;
    
        var moduleDeltas = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
                var current = modulePositions[index];
                var previous = previousPositions[index];

                moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
        }
        var twist = kinematics.toTwist2d(moduleDeltas);
        gyroSim.setAngle( gyro.getAngle() - Units.radiansToDegrees(twist.dtheta)) ;
    }
}
