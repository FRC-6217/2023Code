
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class AutoLevel extends CommandBase {
  /** Creates a new AutoLevel. */
  private TankDrive tankDrive;

  public AutoLevel(TankDrive tankDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    
  }
  enum AutoLevelState {
    INITIALIZE,
    DRIVING_TO,
    GETTING_ON,
    UNBALANCED,
    BALANCING,
    BALANCED,
  };
  AutoLevelState currentState;

  public AutoLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = AutoLevelState.INITIALIZE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoLevelState nextState = currentState;
    switch(currentState) {
      case INITIALIZE:
        nextState = AutoLevelState.DRIVING_TO;
        break;
      case DRIVING_TO:
        if(doDrivingTo()){
          nextState = AutoLevelState.GETTING_ON;
        }
        break;
      case GETTING_ON:
        if (doGettingOn()) {
          nextState = AutoLevelState.UNBALANCED;
        }
        break;
      case UNBALANCED:
        nextState = AutoLevelState.BALANCING;
      case BALANCING:
        if (doBalancing()) {
          nextState = AutoLevelState.BALANCED;
        } else {
          nextState = AutoLevelState.BALANCING;
        }
        break;
      case BALANCED:
        if (isBalanced()) {
          nextState = AutoLevelState.UNBALANCED;
        } else {
          nextState = AutoLevelState.BALANCED;
        }
        break;
      default:
        break;
    }
    currentState = nextState;
    SmartDashboard.putString("autolevelstate", currentState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  // drive forward until we are at 11deg then return true
  private boolean doDrivingTo() {
   
    if(tankDrive.getGyro().getRoll() > 15){
      tankDrive.drive(0, 0);
      tankDrive.resetPosition();
      return true;
    }else{
      tankDrive.drive(-0.3, 0);
    }
    return false;
  }

  // goto 15degs ish or start tilt other way
  private boolean doGettingOn() {
   
    if(tankDrive.getRobotPosition() < -39){
      tankDrive.drive(0, 0);
      return true;
    }else{
      tankDrive.drive(-0.5, 0);
    }
    return false;
  }

  // slowly go back and forward until balanced
  private boolean doBalancing() {



    return isBalanced();
  }

  // check if we are still balanced
  private boolean isBalanced() {
  
    return true;
  }
}
