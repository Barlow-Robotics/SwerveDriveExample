// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drive;


public class RobotContainer {
    private final Drive driveSub = new Drive();

    XboxController driverController; 

  public RobotContainer() {
    configureButtonBindings();

    driveSub.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                driveSub.drive(
                    driverController.getLeftY(),
                    driverController.getLeftX(),
                    driverController.getRightX(),
                    false),
            driveSub));
  }

  private void configureButtonBindings() {
    driverController = new XboxController(1);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
