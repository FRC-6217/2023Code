// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.TankDrive;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;

public class TeleopDrive extends CommandBase  {
  /** Creates a new TeleopDrive. */
  private TankDrive tankDrive;
  private CommandJoystick commandJoystick;
  private boolean isTurningEnabled =true;
  public TeleopDrive(TankDrive tankDrive, CommandJoystick commandJoystick ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    this.commandJoystick = commandJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (commandJoystick.button(OperatorConstants.enableRotationButton).getAsBoolean()){
      isTurningEnabled=true;
    }
    if (commandJoystick.button(OperatorConstants.disableRotationButton).getAsBoolean()){
      isTurningEnabled=false;
    }
    if (isTurningEnabled==true){
      tankDrive.drive(commandJoystick.getY(), commandJoystick.getZ());
    } else{
      tankDrive.drive(commandJoystick.getY(), 0);
    }
    
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
  }
}
