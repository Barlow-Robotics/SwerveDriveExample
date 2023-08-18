// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SwerveModule;

public class DriveRobot extends CommandBase {

    Drive driveSub;
    PS4Controller driverController;
    int ControllerXSpeedID;
    int ControllerYSpeedID;
    int ControllerRotID;
    boolean FieldRelative;
    double DeadBand = 0.01;

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
        double XSpeed = MathUtil.applyDeadband(-driverController.getLeftY(), DeadBand);
        double YSpeed = MathUtil.applyDeadband(-driverController.getLeftX(), DeadBand);
        double Rot = MathUtil.applyDeadband(-driverController.getRightX(), DeadBand);

        XSpeed *= 5700;
        YSpeed *= 5700;
        
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
