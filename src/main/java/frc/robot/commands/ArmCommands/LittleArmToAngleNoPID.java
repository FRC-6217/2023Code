// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem.Arm;

public class LittleArmToAngleNoPID extends CommandBase {
  /** Creates a new LittleArmToAngleNoPID. */
  Arm littleArm; double setpoint; double speed;
  public LittleArmToAngleNoPID(Arm littleArm, double setpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(littleArm);
    this.littleArm = littleArm;
    this.setpoint = setpoint;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    littleArm.armConstantSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    littleArm.armConstantSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(littleArm.getAngle()-setpoint) < 3);
  }
}
