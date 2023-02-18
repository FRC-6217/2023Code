// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class DriveUntilUnBalanced extends CommandBase {
  /** Creates a new Drivetounbalence. */
  TankDrive tankDrive;
  public DriveUntilUnBalanced(TankDrive tankDrive) {
    this.tankDrive = tankDrive;
    addRequirements(tankDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tankDrive.autoDrive(.6, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.tankDrive.getGyro().getPitch()>10;
  }
}
