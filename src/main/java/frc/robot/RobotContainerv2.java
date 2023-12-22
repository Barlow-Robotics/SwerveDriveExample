// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class RobotContainerv2 {

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

    PIDController xPIDController;
    PIDController yPIDController;
    PIDController turnPIDController;

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();;


    /********************************************************************/
    /********************************************************************/

    public final Drive driveSub = new Drive();

    PS4Controller driverController;


    public RobotContainerv2() {
        configureButtonBindings();

        xPIDController = new PIDController(xKP, 0, 0);
        yPIDController = new PIDController(yKP, 0, 0);
        turnPIDController = new PIDController(turnKP, 0, 0);
        SmartDashboard.putData("Auto Mode", autoChooser);
        configurePaths();
        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, driverController, LDALeftStickY, LDALeftStickX, LDARightStickX, true));

        }

    private void configureButtonBindings() {
        driverController = new PS4Controller(1);
    }

    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }
    
    // private void buildAutoOptions() {
    //     autoChooser.setDefaultOption("horizontalLine", "horizontalLine");
    //     autoChooser.addOption("square", "square");
    //     autoChooser.addOption("striaght", "striaght");
    //     SmartDashboard.putData("Path Chooser", autoChooser);
    // }
    private void configurePaths() {
        SmartDashboard.putData("Example Auto", new PathPlannerAuto("test.auto"));

        // Add a button to run pathfinding commands to SmartDashboard
        SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
        new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
        ), 
        0, 
        2.0
        ));
        SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
        new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
        ), 
        0, 
        0
        ));

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m forward of its current position
        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
        Pose2d currentPose = driveSub.getPose();
        
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(
            4.0, 4.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
            ),  
            new GoalEndState(0.0, currentPose.getRotation())
        );

        AutoBuilder.followPathWithEvents(path).schedule();
        }));
    
    }

     
    // InstrumentedSequentialCommandGroup SquareAuto() {
    //     InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();
        
    //     var squarePath = PathPlanner.loadPath("square", 1.0, 2.0, false);

    //     theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = squarePath));
    //     theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(squarePath.getInitialPose())));
    //     theCommand.addCommands(new PPSwerveControllerCommand(
    //         squarePath, 
    //         driveSub::getPose, // Pose supplier
    //         driveSub.kinematics, // SwerveDriveKinematics
    //         xPIDController,
    //         yPIDController,
    //         turnPIDController, 
    //         driveSub::setModuleStates, // Module states consumer
    //         false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //         driveSub));

    //     return theCommand;
    // }




    // InstrumentedSequentialCommandGroup customAuto(String pathName) {
    //     InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();
        
    //     PathPlannerTrajectory autoPath = PathPlanner.loadPath(pathName, 1.0, 2.0, false);

    //     theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = autoPath));
    //     theCmd.addCommands(new InstantCommand(() -> driveSub.resetOdometry(autoPath.getInitialPose())));
    //     theCmd.addCommands(new PPSwerveControllerCommand(
    //         autoPath, 
    //         driveSub::getPose, // Pose supplier
    //         driveSub.kinematics, // SwerveDriveKinematics
    //         xPIDController,
    //         yPIDController,
    //         turnPIDController, 
    //         driveSub::setModuleStates, // Module states consumer
    //         false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //         driveSub));
      
    //     return theCmd;
    // }

    public Command getAutonomousCommand() {
        // String choice = pathChooser.getSelected();
        // return customAuto("horizontalLine");      
        return autoChooser.getSelected();
        // if (choice == "square") {
        //     return customAuto("horizontalLine");      
        // } else if (choice == "horizontalLine") {
        //     return customAuto("horizontalLine");      
        // } else if (choice == "striaght") {
        //     return customAuto("striaghtLine");
        // } else {
        //     System.out.println("Path not choosen");
        //     return null;
        // }
    }
}
