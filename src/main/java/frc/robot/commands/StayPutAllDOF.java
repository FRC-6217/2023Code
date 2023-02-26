// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StayPutCommandConstants;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class StayPutAllDOF extends CommandBase {
  /** Creates a new StayPutAllDOF. */
  TankDrive tankDrive;

  PIDController leftPID = new PIDController(StayPutCommandConstants.P,StayPutCommandConstants.I,StayPutCommandConstants.D);
  PIDController rightPID = new PIDController(StayPutCommandConstants.P,StayPutCommandConstants.I,StayPutCommandConstants.D);

  RobotPosition sPosition;

  public StayPutAllDOF(TankDrive tankDrive) {
    this.tankDrive = tankDrive;
    addRequirements(tankDrive);

    SmartDashboard.putNumber("StayPutP: ", StayPutCommandConstants.P);
    SmartDashboard.putNumber("StayPutI: ", StayPutCommandConstants.I);
    SmartDashboard.putNumber("StayPutD: ", StayPutCommandConstants.D);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPID.reset();
    rightPID.reset();
    sPosition = tankDrive.getStartPosition();
    leftPID.setSetpoint(0);
    rightPID.setSetpoint(0);

    leftPID.setP( SmartDashboard.getNumber("StayPutP: ", StayPutCommandConstants.P));
    leftPID.setI( SmartDashboard.getNumber("StayPutI: ", StayPutCommandConstants.I));
    leftPID.setD( SmartDashboard.getNumber("StayPutD: ", StayPutCommandConstants.D));

    rightPID.setP( SmartDashboard.getNumber("StayPutP: ", StayPutCommandConstants.P));
    rightPID.setI( SmartDashboard.getNumber("StayPutI: ", StayPutCommandConstants.I));
    rightPID.setD( SmartDashboard.getNumber("StayPutD: ", StayPutCommandConstants.D));
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
      tankDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // this command should finish when overridden
  }
}
