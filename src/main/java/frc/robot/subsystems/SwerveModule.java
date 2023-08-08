// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
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

public class SwerveModule {

    /**********************************************************************/
    /***** CONSTANTS *****/

    private static final double WheelRadius = Units.inchesToMeters(2.0);
    private static final int EncoderResolution = 4096;

    private static final double ModuleMaxAngularVelocity = Drive.MaxAngularSpeed;
    private static final double ModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private static final double DriveKP = 1; // Need to change
    private static final double DriveKI = 0;
    private static final double DriveKD = 0;
    private static final double DriveIZone = 0;
    private static final double DriveFF = 0.000015;
    private static final double DriveMinOutput = -1;
    private static final double DriveMaxOutput = 1;
    private static final double MaxRPM = 5700;

    private static final double TurnKP = 1; // Need to change
    private static final double TurnKI = 0;
    private static final double TurnKD = 0;

    private static final double MaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    private static final double MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    private static final double GearRatio = 1; // Need to change
    private static final double MetersPerRevolution = 2 * WheelRadius * Math.PI;
    private static final double VelocityConversionFactor = MetersPerRevolution * 60 * GearRatio; // From RPM to meters/second (?)
    private static final double PositionConversionFactor = MetersPerRevolution * GearRatio; // From revolutions to meters (?)

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

    private final RelativeEncoder driveEncoder;
    private final WPI_CANCoder turnCANCoder;

    public final SparkMaxPIDController drivePIDController;
    public final ProfiledPIDController turnPIDController = new ProfiledPIDController(
            TurnKP,
            TurnKI,
            TurnKD,
            new TrapezoidProfile.Constraints(ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration));

    public final SimpleMotorFeedforward TurnFF = new SimpleMotorFeedforward(0, 0.0); // Need to change these #'s

    public SwerveModule(
            int driveMotorID,
            int turningMotorID,
            int driveEncoderID,
            int turnEncoderID) {

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnCANCoder = new WPI_CANCoder(turnEncoderID, "rio");
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();

        turnCANCoder.configAllSettings(canCoderConfiguration);

        /* FAULT REPORTING */
        CANCoderFaults faults = new CANCoderFaults();
        turnCANCoder.getFaults(faults);
        CANCoderStickyFaults stickyFaults = new CANCoderStickyFaults();
        turnCANCoder.getStickyFaults(stickyFaults);

        drivePIDController = driveMotor.getPIDController();

        setDrivePID();
        setMotorDefaults();
        setEncoderDefaults();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(turnCANCoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getAbsolutePosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turnCANCoder.getAbsolutePosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turnCANCoder.getAbsolutePosition()));

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        final double turnOutput = turnPIDController.calculate(turnCANCoder.getAbsolutePosition(),
                state.angle.getRadians());

        final double turnFF = TurnFF.calculate(turnPIDController.getSetpoint().velocity);

        turnMotor.setVoltage(turnOutput + turnFF);

        if ( RobotBase.isSimulation() ) {
            // double angle = desiredState.angle.getRadians() ;
            double angle = state.angle.getRadians() ;
            CANCoderSimCollection encoderSim = turnCANCoder.getSimCollection() ;

            int rawPosition = 0 ;
            if ( angle < 0) {
                rawPosition = 4096 + (int) ((angle / Math.PI ) * 2048.0) ;
            } else {
                rawPosition = (int) ((angle / Math.PI) * 2048.0)  ;
            }
            encoderSim.setRawPosition( rawPosition ) ;
            Logger.getInstance().recordOutput("CANCoder " + turnCANCoder.getDeviceID(), turnCANCoder.getAbsolutePosition());
            Logger.getInstance().recordOutput("CANCoder Raw " + turnCANCoder.getDeviceID(), rawPosition);
            Logger.getInstance().recordOutput("Module Desired State Angle" + turnCANCoder.getDeviceID(), desiredState.angle.getRadians());
            Logger.getInstance().recordOutput("Module State Angle" + turnCANCoder.getDeviceID(), desiredState.angle.getRadians());
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
        drivePIDController.setOutputRange(DriveMinOutput, DriveMaxOutput);
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
