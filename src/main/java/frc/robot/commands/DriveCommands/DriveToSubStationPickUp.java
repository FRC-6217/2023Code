// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightGamePieceConstants;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.LimeData;

public class DriveToSubStationPickUp extends CommandBase {
  /** Creates a new DriveToSubStationPickUp. */
  TankDrive tankdrive;
  LimeData limedata;

  PIDController forwardpid;

  
  public DriveToSubStationPickUp(TankDrive tankdrive) {
    addRequirements(tankdrive);
    this.tankdrive = tankdrive;
    limedata = tankdrive.getLimeData();
    forwardpid = new PIDController(0, 0, 0);
    SmartDashboard.putNumber("forward Substation: " + " pvalue", LimeLightGamePieceConstants.forwardP); 
    SmartDashboard.putNumber("forward Substation: " + " ivalue", LimeLightGamePieceConstants.forwardI); 
    SmartDashboard.putNumber("forward Substation: " + " dvalue", LimeLightGamePieceConstants.forwardD); 


  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limedata.setPipeline(3); //Maybe put in Execute???
    forwardpid.setP(SmartDashboard.getNumber("forward Substation: " + " pvalue", LimeLightGamePieceConstants.forwardP)); 
    forwardpid.setI(SmartDashboard.getNumber("forward Substation: " + " ivalue", LimeLightGamePieceConstants.forwardI)); 
    forwardpid.setD(SmartDashboard.getNumber("forward Substation: " + " dvalue", LimeLightGamePieceConstants.forwardD)); 

    forwardpid.reset();

    forwardpid.setSetpoint(.21);
    forwardpid.setTolerance(.02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double XSPEED = forwardpid.calculate(limedata.getTa());
    XSPEED = MathUtil.clamp(XSPEED, -LimeLightGamePieceConstants.forwardClamp, LimeLightGamePieceConstants.forwardClamp);
    tankdrive.autoDrive(100*XSPEED, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankdrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardpid.atSetpoint();
  }
}
