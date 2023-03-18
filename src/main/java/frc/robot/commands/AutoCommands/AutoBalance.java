// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class AutoBalance extends CommandBase {
  /** Creates a new AngleDrive. */
  double maxDistanceInches = 48;
  RobotPosition sPosition;
  TankDrive tankDrive;
  double factor = .03;
  public AutoBalance(TankDrive tankDrive) {
  //  SmartDashboard.putNumber("Tilt Factor", factor);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sPosition = tankDrive.getStartPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    @SuppressWarnings("unused")
    RobotPosition cPosition = tankDrive.getRelativePosition(sPosition);
    double tilt = tankDrive.getGyro().getPitch();
   // SmartDashboard.getNumber("Tilt Factor", factor);
    tankDrive.autoDrive(tilt*factor, 0);
    System.out.println(tilt*factor + " " + tilt + " "+ factor);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    tankDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tankDrive.isBalanced();
  }
}
