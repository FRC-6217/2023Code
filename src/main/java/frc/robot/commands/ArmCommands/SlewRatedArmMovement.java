// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;   

import frc.robot.subsystems.ArmSystem.Arm;

public class SlewRatedArmMovement extends CommandBase {
  public SlewRateLimiter slewRateLimiter;
  public Arm arm;

  /** Creates a new SlewRatedArmMovement. */
  public SlewRatedArmMovement(Arm arm) {
    addRequirements(arm);
    this.arm = arm;

 ////   SmartDashboard.putNumber("slew speed " + arm.getArmName(), 0);
  //  SmartDashboard.putNumber(" pos slew " + arm.getArmName(), 0);
  //  SmartDashboard.putNumber(" neg slew " + arm.getArmName(), 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  slewRateLimiter = new SlewRateLimiter(SmartDashboard.getNumber(" pos slew " + arm.getArmName(), 0),
    //                                      SmartDashboard.getNumber(" neg slew " + arm.getArmName(), 0), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // arm.armConstantSpeed(slewRateLimiter.calculate(SmartDashboard.getNumber("slew speed " + arm.getArmName(), 0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //todo add reset ? 
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
