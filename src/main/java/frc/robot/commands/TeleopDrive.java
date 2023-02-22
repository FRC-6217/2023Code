// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.TankDrive;

public class TeleopDrive extends CommandBase {
  /** Creates a new TeleopDrive. */
  private TankDrive tankDrive;

  private CommandJoystick commandJoystick;
 

  public TeleopDrive(TankDrive tankDrive, CommandJoystick commandJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    this.commandJoystick = commandJoystick;
  }

 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //This doesn't let you have brakes off
    tankDrive.enableBreaks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double governer = -commandJoystick.getThrottle() * .5 + .5;
   
    double rotationAllowanceX = Math.abs(commandJoystick.getX()) > .1 ? commandJoystick.getX() : 0;
    double rotationAllowanceY = Math.abs(commandJoystick.getY()) > .1 ? commandJoystick.getY() : 0;
    
    tankDrive.drive(rotationAllowanceY * governer, rotationAllowanceX * governer);
    
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
