// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveRobot extends CommandBase {

    Drive driveSub;
    PS4Controller driverController;

    int ControllerXSpeedID;
    int ControllerYSpeedID;
    int ControllerRotID;

    boolean FieldRelative;

    double DeadBand = 0.08;
    double MaxVelocity = 3.6; //meters per second //value is chosen, not calculated
    double MaxRotVelocity = 2.0; //radians per second
    int MaxRPM = 5676;
    
    public DriveRobot(
        Drive driveSub, 
        PS4Controller driverController, 
        int ControllerXSpeedID, 
        int ControllerYSpeedID, 
        int ControllerRotID,
        boolean FieldRelative) {

        this.driveSub = driveSub;
        this.driverController = driverController;
        this.ControllerXSpeedID = ControllerXSpeedID;
        this.ControllerYSpeedID = ControllerYSpeedID;
        this.FieldRelative = FieldRelative;

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rawX = -driverController.getLeftY() ;
        rawX = 0.2;
        double rawY = driverController.getLeftX() ;
        rawY = 0;
        double rawRot = driverController.getRightX() ;
        rawRot = 0.0 ;

        Logger.getInstance().recordOutput("Raw Yaw Input", rawRot);
        Logger.getInstance().recordOutput("Raw XSpeed", rawX);
        Logger.getInstance().recordOutput("Raw YSpeed", rawY);


        double XSpeed = MathUtil.applyDeadband(rawX, DeadBand);
        double YSpeed = MathUtil.applyDeadband(rawY, DeadBand);
        double Rot = MathUtil.applyDeadband(rawRot, 2*DeadBand);

        XSpeed *= MaxVelocity;
        YSpeed *= MaxVelocity;
        Rot *= MaxRotVelocity;
        
        driveSub.drive(XSpeed, YSpeed, Rot, FieldRelative);

        /* LOGGING */
        Logger.getInstance().recordOutput("Yaw Input", Rot);
        Logger.getInstance().recordOutput("XSpeed", YSpeed);
        Logger.getInstance().recordOutput("YSpeed", XSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
