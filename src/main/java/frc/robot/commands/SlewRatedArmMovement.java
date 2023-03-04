// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSystemConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystem.ARM_SELECTION;

public class SlewRatedArmMovement extends CommandBase {
  public SlewRateLimiter slewRateLimiter;
  public ArmSystem armSystem;
  public ARM_SELECTION selection;
  double direction;
  /** Creates a new SlewRatedArmMovement. */
  public SlewRatedArmMovement(ArmSystem armSystem, ARM_SELECTION selection, double direction) {
    // Use addRequirements() here to declare subsystem dependenciess.
    addRequirements(armSystem);
    this.selection = selection;
    this.armSystem = armSystem;
    this.direction = direction;
    switch(selection){
      case LITTLE_ARM:
        slewRateLimiter = new SlewRateLimiter(ArmSystemConstants.LITTLE_SLEW_POS_RATE,
        ArmSystemConstants.LITTLE_SLEW_NEG_RATE, 0);
        break;
      case BIG_ARM:
        slewRateLimiter = new SlewRateLimiter(ArmSystemConstants.BIG_SLEW_POS_RATE,
        ArmSystemConstants.BIG_SLEW_NEG_RATE, 0);
        break;
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slewRateLimiter.reset(0);
    if(selection == ARM_SELECTION.BIG_ARM)
    armSystem.disableBigArmBreak();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(selection) {
      case LITTLE_ARM:
        armSystem.getLittleArmController().set(direction*slewRateLimiter.calculate(ArmSystemConstants.LITTLE_SLEW_SPEED_RATE));
        break;
      case BIG_ARM:
        armSystem.getBigArmController().set(direction*slewRateLimiter.calculate(ArmSystemConstants.BIG_SLEW_SPEED_RATE));
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
    if(selection == ARM_SELECTION.BIG_ARM)
    armSystem.enableBigArmBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
