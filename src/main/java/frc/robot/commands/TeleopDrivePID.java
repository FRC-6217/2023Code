// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.PIDDriveTrain;

public class TeleopDrivePID extends CommandBase {
  /** Creates a new TeleopDrive. */
  private PIDDriveTrain tankDrive;

  private CommandJoystick commandJoystick;
  private boolean isTurningEnabled = true;

  public TeleopDrivePID(PIDDriveTrain tankDrive, CommandJoystick commandJoystick) {
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
    double governer = -commandJoystick.getThrottle() * .5 + .5;
    double rotationAllowanceZ = Math.abs(commandJoystick.getZ()) > .1 ? commandJoystick.getZ() : 0;
    double rotationAllowanceY = Math.abs(commandJoystick.getY()) > .1 ? commandJoystick.getY() : 0;
    if (isTurningEnabled == true) {
      tankDrive.drive(rotationAllowanceY * governer, rotationAllowanceZ * governer);
    } else {
      tankDrive.drive(rotationAllowanceY * governer, 0);
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
