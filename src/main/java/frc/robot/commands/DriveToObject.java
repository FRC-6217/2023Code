// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.LimeData;

public class DriveToObject extends CommandBase {

  public static enum ObjectType {
    CUBE,
    CONE
  };

  /** Creates a new DriveToObject. */
  TankDrive tankDrive;
  LimeData limedata;
  ObjectType objectType;
  boolean done = false;
  double xspeed;
  double zrotation;
  public DriveToObject(TankDrive tankDrive, ObjectType objectType) {
    this.tankDrive = tankDrive;
    limedata = tankDrive.getLimeData();
    addRequirements(tankDrive);
    this.objectType = objectType;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // setup pipeline
    if(objectType == ObjectType.CUBE){
      limedata.setPipeline(2);  
    }else if(objectType == ObjectType.CONE){
    limedata.setPipeline(1);
    } else{
    limedata.setPipeline(0);
    }
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limedata.getTx() > 4){
      zrotation = -0.3;
    }else if(limedata.getTx() < -4){
      zrotation = 0.3;
    }else if(limedata.getTa() < 7){
      zrotation = 0.0;
      xspeed = 0.3;
    }else {
      done = true;
    }

    tankDrive.autoDrive(xspeed, zrotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
