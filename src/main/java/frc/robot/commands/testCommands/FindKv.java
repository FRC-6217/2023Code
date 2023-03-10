// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.commands.AutoCommands.DriveToDistanceInches;
import frc.robot.subsystems.TankDrive;

public class FindKv extends DriveToDistanceInches {
private double leftVelocitySum = 0;
private double rightVelocitySum = 0;
private double batterySum = 0;
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
    leftVelocitySum = 0;
    rightVelocitySum = 0;
    batterySum = 0;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    counter++;
    leftVelocitySum = tankDrive.getLeftVelocity() + leftVelocitySum;
    rightVelocitySum += tankDrive.getRightVelocity();
    batterySum += RobotController.getBatteryVoltage();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    double leftAverage = leftVelocitySum/counter;
    double rightAverage = rightVelocitySum/counter;
    double batteryAverage = batterySum/counter;

    System.out.println("Left KV: "+ batteryAverage/leftAverage + "\nRight KV: " +batteryAverage/rightAverage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();

  }
}
