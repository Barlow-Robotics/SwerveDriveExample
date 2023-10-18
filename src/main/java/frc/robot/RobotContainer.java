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
    public static final double kPXController = 1.5; // change
    public static final double kPYController = 1.5; // change


    TrapezoidProfile.Constraints thetaConstraintsTrapezoidProfile = new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond,
    AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);

    PathPlannerTrajectory path;
    PIDController xPidController;
    PIDController yPidController;

    final SendableChooser<String> pathChooser = new SendableChooser<String>();



    /********************************************************************/
    /********************************************************************/

    public final Drive driveSub = new Drive();

    PS4Controller driverController;


    public RobotContainer() {
        configureButtonBindings();
        buildAutoOptions();
        xPidController = new PIDController(kPXController, 0, 0);
        yPidController = new PIDController(kPYController, 0, 0);
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
    private void buildAutoOptions() {
        pathChooser.setDefaultOption("Line", "line");
        pathChooser.addOption("BotGo", "BotGo");
        SmartDashboard.putData("Path Chooser", pathChooser);
    }
    InstrumentedSequentialCommandGroup goTop() {
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();
        path = loadPath(pathChooser.getSelected(), 1.0, 3.0, false);

        theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(path.getInitialPose())));
        theCommand.addCommands(new PPSwerveControllerCommand(
            path, 
            driveSub::getPose, // Pose supplier
            driveSub.kinematics, // SwerveDriveKinematics
            xPidController,
            yPidController,
            new PIDController(3.0,0.0,0.0), 
            driveSub::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSub));
        theCommand.addCommands(new InstantCommand(() -> driveSub.drive(0.0,0.0,0.0,true)));
        return theCommand;
    }



    public Command getAutonomousCommand() {

        // PathPlannerTrajectory traj2 = PathPlanner.generatePath(
        //     new PathConstraints(3, 16), 
        //     new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
        //     new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
        //     new PathPoint(new Translation2d(10.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
        //     new PathPoint(new Translation2d(15.0, 7.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-45)) // position, heading(direction of travel), holonomic rotation
        // );
        // Logger.getInstance().recordOutput("PP Auto Path", traj2);
        
        String choice = pathChooser.getSelected();
        if (choice == "BotGo") {
            return goTop();
        }
        else {
            System.out.println("Choose not choosen");
            return null;
        }

    }
}
