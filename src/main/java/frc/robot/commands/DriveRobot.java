// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveRobot extends CommandBase {

    Drive driveSub;
    Joystick driverController;
    int controllerXSpeedID;
    int controllerYSpeedID;
    int controllerRotID;
    boolean FieldRelative;

    public DriveRobot(
        Drive driveSub, 
        Joystick driverController, 
        int ControllerXSpeedID, 
        int ControllerYSpeedID, 
        int ControllerRotID,
        boolean FieldRelative) {

        this.driveSub = driveSub;
        this.driverController = driverController;
        this.controllerXSpeedID = ControllerXSpeedID;
        this.controllerYSpeedID = ControllerYSpeedID;
        this.FieldRelative = FieldRelative;

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double XSpeed = driverController.getRawAxis(controllerXSpeedID);
        if (Math.abs(XSpeed) < 0.005) {
            XSpeed = 0.0;
        }

        double YSpeed = driverController.getRawAxis(controllerYSpeedID);
        if (Math.abs(YSpeed) < 0.005) {
            YSpeed = 0.0;
        }

        double Rot = driverController.getRawAxis(controllerRotID);
        if (Math.abs(Rot) < 0.005) {
            Rot = 0.0;
        }

        driveSub.drive(XSpeed, YSpeed, Rot, FieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}