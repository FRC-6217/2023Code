// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystem.ArmPosition;
import frc.robot.subsystems.ArmSystem.Position;

public class ArmPositionsAuto extends CommandBase {
  /** Creates a new ArmPositionsAuto. */
  ArmSystem armSystem;
  Position goalPosition;
  PIDController pidLittleArm;
  PIDController pidBigArm;
  public ArmPositionsAuto(ArmSystem armSystem, Position goalPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSystem);
    this.armSystem = armSystem;
    this.goalPosition = goalPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSystem.disableBigArmBreak();
    pidBigArm.reset();
    pidLittleArm.reset();

    pidBigArm.setP( SmartDashboard.getNumber("BigArmP: ", Constants.ArmSystemConstants.BigArmAngle.Pvalue));
    pidBigArm.setI( SmartDashboard.getNumber("BigArmI: ", Constants.ArmSystemConstants.BigArmAngle.Ivalue));
    pidBigArm.setD( SmartDashboard.getNumber("BigArmD: ", Constants.ArmSystemConstants.BigArmAngle.Dvalue));

    pidLittleArm.setP( SmartDashboard.getNumber("StayPutP: ", Constants.ArmSystemConstants.LittleArmAngle.Pvalue));
    pidLittleArm.setI( SmartDashboard.getNumber("StayPutI: ", Constants.ArmSystemConstants.LittleArmAngle.Ivalue));
    pidLittleArm.setD( SmartDashboard.getNumber("StayPutD: ", Constants.ArmSystemConstants.LittleArmAngle.Dvalue));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Position nextPosition;
    Position currentPosition = armSystem.getCurrentPosition();
    if(currentPosition.ordinal() > goalPosition.ordinal()){
      nextPosition = Position.values()[currentPosition.ordinal() - 1];
    }else if (currentPosition.ordinal() < goalPosition.ordinal()){
      nextPosition = Position.values()[currentPosition.ordinal() + 1];
    }else{
      nextPosition = currentPosition;
    }
  
    System.out.println("Current Position: " + currentPosition + "   Next Position: " + nextPosition + "   Goal Position: " + goalPosition);
    
    if(currentPosition != nextPosition){
      ArmPosition setpoints = armSystem.getSetPointsFor(goalPosition);
      double littleArmGoalPosition = setpoints.littleArmSetPoint;
      double bigArmGoalPosition = setpoints.bigArmSetPoint;

      double littleInput = pidLittleArm.calculate(armSystem.getLittleArmPositon(),littleArmGoalPosition);
      double bigInput = pidLittleArm.calculate(armSystem.getBigArmPosition(),bigArmGoalPosition);

      armSystem.setBigArm(bigInput);
      armSystem.setLittleArm(littleInput);
    }else{
      armSystem.setBigArm(0);
      armSystem.setLittleArm(0);
      armSystem.setPostion(nextPosition);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSystem.enableBigArmBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSystem.getCurrentPosition()== goalPosition;
  }

 

}
