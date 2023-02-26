// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class FindKs extends CommandBase {
  /** Creates a new FindKs. */
  TankDrive tankdrive;

  double leftvoltage;
  double rightvoltage;
  double voltageIncrement;
  RobotPosition startingPosition;
  int counterLeft = 0;
  int counterRight = 0;
  double aveVoltageLeft = 0;
  double aveVoltageRight = 0;
  boolean leftdone = false;
  boolean rightdone = false;

  public FindKs(TankDrive tankDrive) {
    this.tankdrive = tankDrive;
    addRequirements(tankDrive);
    leftvoltage = 0;
    rightvoltage = 0;
    voltageIncrement = 0.001;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftdone = false;
    rightdone = false;
    startingPosition = tankdrive.getStartPosition();
    leftvoltage = 0;
    rightvoltage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tankdrive.setDriveVoltage(leftvoltage, rightvoltage);
    System.out.println("left voltage: " + leftvoltage + " rightvoltage: " + rightvoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aveVoltageLeft /= counterLeft;
    aveVoltageRight /= counterRight;
    counterLeft = 0;
    counterRight = 0;
    //tankDrive.stopDrive();
    System.out.println("ave: leftvoltage: " + aveVoltageLeft + " rightvoltage: " + aveVoltageRight);
    aveVoltageLeft = 0;
    aveVoltageRight = 0;
    leftdone = false;
    rightdone = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotPosition currentposition = tankdrive.getRelativePosition(startingPosition);
    if (currentposition.leftposition > 0) {
      leftdone = true;
    } else {
      leftvoltage = voltageIncrement + leftvoltage;
    }
    if (currentposition.rightposition > 0) {
      rightdone = true;
    } else {
      rightvoltage = voltageIncrement + rightvoltage;
    }
    if (rightdone && leftdone) {
      counterLeft = counterLeft + 1;
      counterRight = counterRight + 1;

      aveVoltageLeft += leftvoltage;
      aveVoltageRight += rightvoltage;

      if (counterLeft == 10) {
        return true;
      } else {
        this.initialize();
      }
    }
    return false;
  }

  
}
