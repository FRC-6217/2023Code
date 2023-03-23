// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightGamePieceConstants;
import frc.robot.commands.AutoCommands.DriveToDistanceInches;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.LimeData;

public class DriveUpToTheSubstation extends CommandBase {
  /** Creates a new DriveUpToTheSubstation. */
  TankDrive tankDrive;
  LimeData limeDatal;

  boolean done = false;

  PIDController rotatePID;

  public DriveUpToTheSubstation(TankDrive tankDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tankDrive = tankDrive;
    limeDatal = tankDrive.getLimeData();
    addRequirements(tankDrive);
    rotatePID = new PIDController(0, 0, 0);
    
    SmartDashboard.putNumber("rotate gamepiecepid" + " pvalue", LimeLightGamePieceConstants.rotateP); 
    SmartDashboard.putNumber("rotate gamepiecepid" + " ivalue", LimeLightGamePieceConstants.rotateI); 
    SmartDashboard.putNumber("rotate gamepiecepid" + " dvalue", LimeLightGamePieceConstants.rotateD);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotatePID.setP(SmartDashboard.getNumber("rotate gamepiecepid" + " pvalue", LimeLightGamePieceConstants.rotateP)); 
    rotatePID.setI(SmartDashboard.getNumber("rotate gamepiecepid" + " ivalue", LimeLightGamePieceConstants.rotateI)); 
    rotatePID.setD(SmartDashboard.getNumber("rotate gamepiecepid" + " dvalue", LimeLightGamePieceConstants.rotateD)); 

    rotatePID.reset();

    SmartDashboard.putNumber("Substation Rotate Setpoint: ", 13);

    rotatePID.setSetpoint(SmartDashboard.getNumber("Substation Rotate Setpoint: ", 0));
    rotatePID.setTolerance(LimeLightGamePieceConstants.rotateTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = -rotatePID.calculate(limeDatal.getTx());
    rotate = MathUtil.clamp(rotate, -LimeLightGamePieceConstants.rotateClamp, LimeLightGamePieceConstants.rotateClamp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new DriveToDistanceInches(tankDrive, 28, .5).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
