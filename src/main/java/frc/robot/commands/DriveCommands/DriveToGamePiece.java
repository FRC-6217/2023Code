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

public class DriveToGamePiece extends CommandBase {
  /** Creates a new Drivetogamepeace. */
  public static enum GamePiece {
    cube,cone,both
  }
  TankDrive tankdrive; 
  GamePiece piece;
  LimeData limedata;
  PIDController forwardpid;
  PIDController rotatepid;
  public DriveToGamePiece(TankDrive tankdrive, GamePiece piece) {
  addRequirements(tankdrive);
    // Use addRequirements() here to declare subsystem dependencies.
    this.tankdrive = tankdrive;
    this.piece = piece;
    limedata = tankdrive.getLimeData();
    forwardpid = new PIDController(0, 0, 0);
    rotatepid = new PIDController(0, 0, 0);
    SmartDashboard.putNumber("forward gamepiecepid" + " pvalue", LimeLightGamePieceConstants.forwardP); 
    SmartDashboard.putNumber("forward gamepiecepid" + " ivalue", LimeLightGamePieceConstants.forwardI); 
    SmartDashboard.putNumber("forward gamepiecepid" + " dvalue", LimeLightGamePieceConstants.forwardD); 
    SmartDashboard.putNumber("rotate gamepiecepid" + " pvalue", LimeLightGamePieceConstants.rotateP); 
    SmartDashboard.putNumber("rotate gamepiecepid" + " ivalue", LimeLightGamePieceConstants.rotateI); 
    SmartDashboard.putNumber("rotate gamepiecepid" + " dvalue", LimeLightGamePieceConstants.rotateD); 
   
    
  
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardpid.setP(SmartDashboard.getNumber("forward gamepiecepid" + " pvalue", LimeLightGamePieceConstants.forwardP)); 
    forwardpid.setI(SmartDashboard.getNumber("forward gamepiecepid" + " ivalue", LimeLightGamePieceConstants.forwardI)); 
    forwardpid.setD(SmartDashboard.getNumber("forward gamepiecepid" + " dvalue", LimeLightGamePieceConstants.forwardD)); 
    rotatepid.setP(SmartDashboard.getNumber("rotate gamepiecepid" + " pvalue", LimeLightGamePieceConstants.rotateP)); 
    rotatepid.setI(SmartDashboard.getNumber("rotate gamepiecepid" + " ivalue", LimeLightGamePieceConstants.rotateI)); 
    rotatepid.setD(SmartDashboard.getNumber("rotate gamepiecepid" + " dvalue", LimeLightGamePieceConstants.rotateD)); 

    forwardpid.reset();
    rotatepid.reset();

    rotatepid.setSetpoint(LimeLightGamePieceConstants.rotateSetpoint);
    rotatepid.setTolerance(LimeLightGamePieceConstants.rotateTolerance);
    forwardpid.setSetpoint(LimeLightGamePieceConstants.coneForwardSetpoint);
    forwardpid.setTolerance(LimeLightGamePieceConstants.forwardTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = -rotatepid.calculate(limedata.getTx());
    rotate = MathUtil.clamp(rotate, -LimeLightGamePieceConstants.rotateClamp, LimeLightGamePieceConstants.rotateClamp);
    double XSPEED = forwardpid.calculate(limedata.getTa());
    XSPEED = MathUtil.clamp(XSPEED, -LimeLightGamePieceConstants.forwardClamp, LimeLightGamePieceConstants.forwardClamp);
    tankdrive.autoDrive(XSPEED, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankdrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
