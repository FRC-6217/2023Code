// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class GoToAngle extends CommandBase {
  RobotPosition startPosition; 
  TankDrive tankDrive;
  double angle; 
  double speed; 
    /** Creates a new InchesDrive. */
    public GoToAngle(TankDrive tankDrive, double angle, double speed) {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(tankDrive);
      this.tankDrive = tankDrive;
      this.angle = angle;
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
    if(angle > 0){
      tankDrive.autoDrive(0, Math.abs(speed));
      }else{
      tankDrive.autoDrive(0, -Math.abs(speed));
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
    if(angle >= 0){
      if(cPosition.angle >= angle){
      return true;
      }
    }else{
      if(cPosition.angle <= angle){
        return true;
      }
    }
    return false;
  }
}
