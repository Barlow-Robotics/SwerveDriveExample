// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.subsystems.Drive;

/* OVERALL TO DO
* EHP need to turn drive into a command outside of RobotContainer -- done
* EHP add the field into the simulation
* EHP add simulation files and code
* EHP set up networktables/smartdashboard
* EHP auto???
*/

public class RobotContainer {

    /********************************************************************/
    /***** CONSTANTS *****/

    public static final int LDALeftStickX = 0; // LDA = Logitech Dual Action
    public static final int LDALeftStickY = 1;
    public static final int LDARightStickX = 2;
    public static final int LDARightStickY = 3;
    public static final int LDALeftTrigger = 7;
    public static final int LDARightTrigger = 8;
    public static final int LDAButtonA = 2;
    public static final int LDAButtonB = 3;
    public static final int LDAButtonX = 1;
    public static final int LDAButtonY = 4;
    public static final int LDALeftBumper = 5;
    public static final int LDARightBumper = 6;
    public static final int LDABackButton = 9;
    public static final int LDAStartButton = 10;
    public static final int LDALeftStick = 11;
    public static final int LDARightStick = 12;
    public static final double LDAForwardAxisAttenuation = -0.5;
    public static final double LDALateralAxisAttenuation = 0.5;
    public static final double LDAYawAxisAttenuation = 0.5;


    public static final double xKP = 1.5; // change
    public static final double yKP = 1.5; // change
    public static final double turnKP = 0.5;


    TrapezoidProfile.Constraints thetaConstraintsTrapezoidProfile = new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond,
    AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);

    PathPlannerTrajectory currentTrajectory = null;
    PathPlannerTrajectory botGoAround;
    PathPlannerTrajectory botGo;

    PIDController xPIDController;
    PIDController yPIDController;
    PIDController turnPIDController;

    final SendableChooser<String> pathChooser = new SendableChooser<String>();


    /********************************************************************/
    /********************************************************************/

    public final Drive driveSub = new Drive();

    PS4Controller driverController;


    public RobotContainer() {
        configureButtonBindings();
        buildAutoOptions();

        xPIDController = new PIDController(xKP, 0, 0);
        yPIDController = new PIDController(yKP, 0, 0);
        turnPIDController = new PIDController(turnKP, 0, 0);

        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, driverController, LDALeftStickY, LDALeftStickX, LDARightStickX, true));
    }

    private void configureButtonBindings() {
        driverController = new PS4Controller(1);
    }

    private PathPlannerTrajectory loadPath(String name, double velocity, double accel, boolean reverse) {
        PathPlannerTrajectory temp = PathPlanner.loadPath(
                        name,
                        new PathConstraints(velocity, accel),
                        reverse);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(temp, DriverStation.getAlliance());
    }

    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }
    
    private void buildAutoOptions() {
        pathChooser.setDefaultOption("square", "square");
        pathChooser.addOption("BotGo", "BotGo");
        SmartDashboard.putData("Path Chooser", pathChooser);
    }

    InstrumentedSequentialCommandGroup BotGoAround() {
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        botGoAround = loadPath("BotGoAround", 4.0, 3.0, false);

        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = botGoAround));
        theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(botGoAround.getInitialPose())));
        theCommand.addCommands(new PPSwerveControllerCommand(
            botGoAround, 
            driveSub::getPose, // Pose supplier
            driveSub.kinematics, // SwerveDriveKinematics
            xPIDController,
            yPIDController,
            new PIDController(3.0,0.0,0.0), 
            driveSub::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSub));

        return theCommand;
    }
    
    InstrumentedSequentialCommandGroup BotGo() {
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();
        
        botGo = loadPath("square", 4.0, 3.0, false);

        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = botGo));
        theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(botGo.getInitialPose())));
        theCommand.addCommands(new PPSwerveControllerCommand(
            botGo, 
            driveSub::getPose, // Pose supplier
            driveSub.kinematics, // SwerveDriveKinematics
            xPIDController,
            yPIDController,
            turnPIDController, 
            driveSub::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSub));

        return theCommand;
    }
    InstrumentedSequentialCommandGroup customCommand(PathPlannerTrajectory path) {
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();
        
        botGo = path;
        var ppCommand = new PPSwerveControllerCommand(
            botGo, 
            driveSub::getPose, // Pose supplier
            driveSub.kinematics, // SwerveDriveKinematics
            xPIDController,
            yPIDController,
            turnPIDController, 
            driveSub::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSub);
        ppCommand.setLoggingCallbacks(null, null, null, null);
        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = botGo));
        theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(botGo.getInitialPose())));
        theCommand.addCommands(ppCommand);
        return theCommand;
    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            new PathConstraints(2, 8), 
            new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(2.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(2.0, 2.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(0.0, 2.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
        );

        String choice = pathChooser.getSelected();
        if (choice == "square") {
            return BotGo();
        }
        else if (choice == "custom") {
            return customCommand(traj2);
        }
        else {
            System.out.println("Path not choosen");
            return null;
        }

    }
}
