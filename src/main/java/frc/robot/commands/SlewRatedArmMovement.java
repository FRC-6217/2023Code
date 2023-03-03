// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystem.ARM_SELECTION;

public class SlewRatedArmMovement extends CommandBase {
  public SlewRateLimiter slewRateLimiter;
  public ArmSystem armSystem;
  public ARM_SELECTION selection;
  /** Creates a new SlewRatedArmMovement. */
  public SlewRatedArmMovement(ArmSystem armSystem, ARM_SELECTION selection) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSystem);
    this.selection = selection;
    this.armSystem = armSystem;

    SmartDashboard.putNumber("slew speed " + selection, 0);
    SmartDashboard.putNumber(" pos slew " + selection, 0);
    SmartDashboard.putNumber(" neg slew " + selection, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slewRateLimiter = new SlewRateLimiter(SmartDashboard.getNumber(" pos slew " + selection, 0),
                                          SmartDashboard.getNumber(" neg slew " + selection, 0), 0);
    slewRateLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(selection) {
      case LITTLE_ARM:
        armSystem.getLittleArmController().set(slewRateLimiter.calculate(SmartDashboard.getNumber("slew speed " + selection, 0)));
        break;
      case BIG_ARM:
        armSystem.getBigArmController().set(slewRateLimiter.calculate(SmartDashboard.getNumber("slew speed " + selection, 0)));
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //todo add reset ? 
    switch(selection) {
      case LITTLE_ARM:
        armSystem.getLittleArmController().set(0);
        break;
      case BIG_ARM:
        armSystem.getBigArmController().set(0);
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
