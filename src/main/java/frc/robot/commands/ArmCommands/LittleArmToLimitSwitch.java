// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class LittleArmToLimitSwitch extends CommandBase {
  /** Creates a new LittleArmToLimitSwitch. */
  ArmSystem armSystem;
  public LittleArmToLimitSwitch(ArmSystem armSystem) {
    this.armSystem = armSystem;
    addRequirements(armSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSystem.littleArm.set(.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSystem.littleArm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSystem.getLittleArmController().getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }
}
