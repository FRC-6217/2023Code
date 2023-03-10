// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
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
    SmartDashboard.putNumber("Rotation Governer Multiple: ", OperatorConstants.defaultRotationGoverner);
  }

 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //This doesn't let you have brakes off
   // tankDrive.enableBreaks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double governer = -commandJoystick.getThrottle() * .5 + .5;

    double rotationGoverner = SmartDashboard.getNumber("Rotation Governer Multiple: ", OperatorConstants.defaultRotationGoverner);

   
    double rotation = Math.abs(commandJoystick.getZ()) > OperatorConstants.deadBandX ? commandJoystick.getZ() : 0;
    double speed = Math.abs(commandJoystick.getY()) > OperatorConstants.deadBandY ? commandJoystick.getY() : 0;

    double rotateMultipler = 1;
    if (commandJoystick.button(1).getAsBoolean()) {
      rotateMultipler = 0.3;
    }
    tankDrive.drive(speed * governer, rotation *( governer* rotationGoverner) * rotateMultipler);
    
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
  }
}
