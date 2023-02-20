// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class DriveUntilUnBalanced extends CommandBase {
  public static enum Direction{
    forwards, backwards
  }
  /** Creates a new Drivetounbalence. */
  TankDrive tankDrive;
  Direction direction;
  public DriveUntilUnBalanced(TankDrive tankDrive, Direction direction) {
    this.tankDrive = tankDrive;
    this.direction = direction;
    addRequirements(tankDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction == Direction.forwards)
    tankDrive.autoDrive(.7, 0);
    else
    tankDrive.autoDrive(-.7, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.tankDrive.getGyro().getPitch())>10;
  }
}
