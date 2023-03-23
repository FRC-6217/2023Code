// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class DriveToDistanceInches extends CommandBase {
RobotPosition startPosition; 
public TankDrive tankDrive;
public double distanceMeters; 
public double speed; 
double direction;
  /** Creates a new InchesDrive. */
  public DriveToDistanceInches(TankDrive tankDrive, double inches, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    if(inches < 0){
      direction = -1;
    }
    else{
      direction = 1;
    }
    this.tankDrive = tankDrive;
    this.distanceMeters = Units.inchesToMeters(Math.abs(inches));
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
  tankDrive.autoDrive(Math.abs(speed)*direction, 0);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getName() + " cancelled");
    tankDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotPosition cPosition = tankDrive.getRelativePosition(startPosition);
    return Math.abs(cPosition.averagePosition) > distanceMeters;
  }
}
