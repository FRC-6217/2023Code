// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class InchesDrive extends CommandBase {
RobotPosition startPosition; 
TankDrive tankDrive;
double inches; 
double speed; 
  /** Creates a new InchesDrive. */
  public InchesDrive(TankDrive tankDrive, double inches, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    this.inches = inches;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = tankDrive.getStartPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(inches > 0){
    tankDrive.autoDrive(Math.abs(speed), 0);
    }else{
    tankDrive.autoDrive(-Math.abs(speed), 0);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotPosition cPosition = tankDrive.getRelativePosition(startPosition);
    if(inches >= 0){
      if(cPosition.averagePosition >= inches){
      return true;
      }
    }else{
      if(cPosition.averagePosition <= inches){
        return true;
      }
    }
    return false;
  }
}
