// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class StayPutAllDOF extends CommandBase {
  /** Creates a new StayPutAllDOF. */
  TankDrive tankDrive;

  PIDController leftPID = new PIDController(0.2000,0,0);
  PIDController rightPID = new PIDController(0.2000, 0, 0);

  RobotPosition sPosition;

  public StayPutAllDOF(TankDrive tankDrive) {
    this.tankDrive = tankDrive;
    addRequirements(tankDrive);

    SmartDashboard.putNumber("StayPutP: ", 0.2000);
    SmartDashboard.putNumber("StayPutI: ", 0);
    SmartDashboard.putNumber("StayPutD: ", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPID.reset();
    rightPID.reset();
    sPosition = tankDrive.getStartPosition();
    leftPID.setSetpoint(0);
    rightPID.setSetpoint(0);

    leftPID.setP( SmartDashboard.getNumber("StayPutP: ", 0));
    leftPID.setI( SmartDashboard.getNumber("StayPutI: ", 0));
    leftPID.setD( SmartDashboard.getNumber("StayPutD: ", 0));

    rightPID.setP( SmartDashboard.getNumber("StayPutP: ", 0));
    rightPID.setI( SmartDashboard.getNumber("StayPutI: ", 0));
    rightPID.setD( SmartDashboard.getNumber("StayPutD: ", 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotPosition cPosition = tankDrive.getRelativePosition(sPosition);
    double leftOutput = leftPID.calculate(cPosition.leftposition);
    double rightOutput = rightPID.calculate(cPosition.rightposition);

    tankDrive.autoDriveDifferential(leftOutput, rightOutput);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      tankDrive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // this command should finish when overridden
  }
}
