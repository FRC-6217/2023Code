// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommands.DriveToDistanceInches;
import frc.robot.subsystems.TankDrive;

/** Add your docs here. */
public class DriveToDistanceInchesTuning  extends DriveToDistanceInches{
    
    public DriveToDistanceInchesTuning(TankDrive tankDrive, double inches, double speed) {
        super(tankDrive, inches, speed);
        SmartDashboard.putNumber("DriveToInches Tuning Inches: ", inches);
        SmartDashboard.putNumber("DriveToInches Tuning Speed: ", speed);
    }

    public void initialize(){
        double newInches = SmartDashboard.getNumber("DriveToInches Tuning Inches: ", 0);
        double newSpeed =  SmartDashboard.getNumber("DriveToInches Tuning Speed: ", speed);

        this.distanceMeters = Units.inchesToMeters(Math.abs(newInches));
        this.speed = newSpeed;
        super.initialize();
    }

}
