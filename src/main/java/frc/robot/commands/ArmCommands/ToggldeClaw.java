// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PneumaticConstants.Claw;
import frc.robot.subsystems.ArmSystem;

public class ToggldeClaw extends CommandBase {
  ArmSystem armSystem;
  public static enum cState{
    OPEN,
    CLOSED,
  }
  cState c;
  /** Creates a new ToggldeClaw. */
  public ToggldeClaw(ArmSystem armSystem, cState c) {
    this.armSystem = armSystem;
    addRequirements(armSystem);
    this.c = c;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(c == cState.OPEN){
      armSystem.getClaw().set(Value.kReverse);
    }else{
      armSystem.getClaw().set(Value.kForward);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
