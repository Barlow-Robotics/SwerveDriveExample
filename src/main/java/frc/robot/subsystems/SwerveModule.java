// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

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

  private static final double DriveFeedForwardKS = 1; // EHP change
  private static final double DriveFeedForwardKV = 3; // EHP change
  private static final double TurnFeedForwardKS = 1; // EHP change
  private static final double TurnFeedForwardKV = 0.5; // EHP change

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
  private final Spark turnMotor;

  private final Encoder driveEncoder;
  private final Encoder turnEncoder;

  private final PIDController drivePIDController = new PIDController(DriveKP, DriveKI, DriveKD);
  private final ProfiledPIDController turnPIDController = // EHP change
            new ProfiledPIDController(
                    TurnKP,
                    TurnKI,
                    TurnKD,
                    new TrapezoidProfile.Constraints(ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration));


  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderChannels,
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    driveMotor = new Spark(driveMotorChannel);
    turnMotor = new Spark(turningMotorChannel);

    driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);
    driveEncoder.setDistancePerPulse(kDriveEncoderDistancePerPulse);
    driveEncoder.setReverseDirection(driveEncoderReversed);

    turnEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);
    turnEncoder.setDistancePerPulse(kTurningEncoderDistancePerPulse);
    turnEncoder.setReverseDirection(turningEncoderReversed);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(turnEncoder.getDistance()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getDistance(), new Rotation2d(turnEncoder.getDistance()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getDistance()));

    final double driveOutput = drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);
    final double turnOutput = turnPIDController.calculate(turnEncoder.getDistance(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turnMotor.set(turnOutput);
  }

  public void resetEncoders() {
    driveEncoder.reset();
    turnEncoder.reset();
  }
}
