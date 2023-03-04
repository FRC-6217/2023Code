// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystem.ARM_SELECTION;

public class SetArmZero extends CommandBase {
  /** Creates a new SetArmZero. */
  ArmSystem armSystem;
  ARM_SELECTION a;
  boolean done = false;
  public SetArmZero(ArmSystem armSystem, ARM_SELECTION a) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSystem = armSystem;
    addRequirements(armSystem);
    this.a = a;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(a){
      case BIG_ARM:
        armSystem.bigArmBackward();
        break;
      case LITTLE_ARM:
        armSystem.littleArmForward();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(a == ARM_SELECTION.BIG_ARM){
      armSystem.bigArmOff();
    }else if(a == ARM_SELECTION.LITTLE_ARM){
      armSystem.littleArmOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(a == ARM_SELECTION.BIG_ARM){
      return armSystem.bigArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    } else if (a == ARM_SELECTION.LITTLE_ARM){
      return armSystem.littleArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }else{
      return false;
    }
  }
}
