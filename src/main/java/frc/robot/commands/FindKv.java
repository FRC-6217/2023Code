// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCommands.InchesDrive;
import frc.robot.subsystems.TankDrive;

public class FindKv extends InchesDrive {
private double leftVelocitySum = 0;
private double rightVelocitySum = 0;
private int counter = 0;

  
  /** Creates a new FindKv. */
  public FindKv(TankDrive tankDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(tankDrive, 20*12, 1);
    addRequirements(tankDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    counter++;
    leftVelocitySum = tankDrive.getLeftVelocity() + leftVelocitySum;
    rightVelocitySum += tankDrive.getRightVelocity();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    double leftAverage = leftVelocitySum/counter;
    double rightAverage = rightVelocitySum/counter;
    System.out.println("Left KV: "+ leftAverage/RobotController.getBatteryVoltage() + "\nRight KV: " +rightAverage/RobotController.getBatteryVoltage());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();

  }
}
