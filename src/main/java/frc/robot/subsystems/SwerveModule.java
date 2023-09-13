// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class SwerveModule {

    /**********************************************************************/
    /***** CONSTANTS *****/

    private static final double WheelRadius = Units.inchesToMeters(2.0);
    private static final double WheelCircumference = Units.inchesToMeters(2.0 * WheelRadius * Math.PI);

    /* DRIVE ENCODER */
    private static final double DriveKP = 1; // Need to change
    private static final double DriveKI = 0;
    private static final double DriveKD = 0;
    private static final double DriveIZone = 0;
    private static final double DriveFF = 0.000015;

    private static final double GearRatio = 8.14;
    private static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute / GearRatio;
    // private static final double PositionConversionFactor = WheelCircumference /
    // GearRatio;
    private static final double PositionConversionFactor = 1000 * WheelCircumference / GearRatio;
    private static final double MaxRPM = 5700;
    public static final double MaxVelocityPerSecond = MaxRPM * VelocityConversionFactor;

    /* TURN ENCODER */
    private static final int CANCoderResolution = 4096;

    private static final double TurnKP = 1; // Need to change
    private static final double TurnKI = 0;
    private static final double TurnKD = 0;
    private static final double ModuleMaxAngularVelocity = Drive.MaxAngularSpeed;
    private static final double ModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    /**********************************************************************/
    /**********************************************************************/

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final WPI_CANCoder turnEncoder;

    private final SparkMaxPIDController drivePIDController;
    private final SparkMaxPIDController turnPIDController;

    private final SimpleMotorFeedforward TurnFF = new SimpleMotorFeedforward(0, 0.0); // Need to change these #'s

    public SwerveModule(
            int driveMotorID,
            int turningMotorID,
            int turnEncoderID) {

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new WPI_CANCoder(turnEncoderID, "rio");

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        turnEncoder.configAllSettings(canCoderConfiguration);

        drivePIDController = driveMotor.getPIDController();
        turnPIDController = turnMotor.getPIDController();

        setDrivePID();
        setMotorDefaults();
        setEncoderDefaults();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turnEncoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turnEncoder.getAbsolutePosition()));

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        // final double turnOutput = turnPIDController.calculate(turnEncoder.getAbsolutePosition(),
        //         state.angle.getRadians());

        // final double turnFF = TurnFF.calculate(turnPIDController.getSetpoint().velocity);

        // turnMotor.setVoltage(turnOutput + turnFF);

        turnPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
        
        if (RobotBase.isSimulation()) {
            // double angle = desiredState.angle.getRadians() ;
            double angle = state.angle.getRadians();
            CANCoderSimCollection encoderSim = turnEncoder.getSimCollection();

            int rawPosition = 0;
            if (angle < 0) {
                rawPosition = 4096 + (int) ((angle / Math.PI) * 2048.0);
            } else {
                rawPosition = (int) ((angle / Math.PI) * 2048.0);
            }
            encoderSim.setRawPosition(rawPosition);
            Logger.getInstance().recordOutput("CANCoder " + turnEncoder.getDeviceID(),
                    turnEncoder.getAbsolutePosition());
            Logger.getInstance().recordOutput("CANCoder Raw " + turnEncoder.getDeviceID(), rawPosition);
            Logger.getInstance().recordOutput("Module Desired State Angle" + turnEncoder.getDeviceID(),
                    desiredState.angle.getRadians());
            Logger.getInstance().recordOutput("Module State Angle" + turnEncoder.getDeviceID(),
                    desiredState.angle.getRadians());
        }
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void setDrivePID() {
        drivePIDController.setP(DriveKP);
        drivePIDController.setI(DriveKI);
        drivePIDController.setD(DriveKD);
        drivePIDController.setIZone(DriveIZone);
        drivePIDController.setFF(DriveFF);
    }

    public void setMotorDefaults() {
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setEncoderDefaults() {
        driveEncoder.setVelocityConversionFactor(VelocityConversionFactor);
        driveEncoder.setPositionConversionFactor(PositionConversionFactor);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNEO(1));
    }
}
