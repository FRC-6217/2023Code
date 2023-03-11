package frc.robot.commands.testCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestCoolBeans;

public class TestCommandCoolBeans extends CommandBase {
  
  private TestCoolBeans t;

  SparkMaxPIDController pid;
  double p, i, d, f;

  public TestCommandCoolBeans(TestCoolBeans t) {
    this.t = t;
    pid = t.getController().getPIDController();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(t);

    SmartDashboard.putNumber("pcbcb", 0);
    SmartDashboard.putNumber("icb", 0);
    SmartDashboard.putNumber("dcb", 0);
    SmartDashboard.putNumber("fcbcb", 0);
    SmartDashboard.putNumber("scbcb", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getNumber("pcbcb", 0) != pid.getP()) {
      pid.setP(SmartDashboard.getNumber("pcbcb", 0));
    }
    if (SmartDashboard.getNumber("icb", 0) != pid.getI()) {
      pid.setI(SmartDashboard.getNumber("icb", 0));
    }
    if (SmartDashboard.getNumber("dcb", 0) != pid.getD()) {
      pid.setD(SmartDashboard.getNumber("dcb", 0));
    }
    if (SmartDashboard.getNumber("fcbcb", 0) != pid.getFF()) {
      pid.setFF(SmartDashboard.getNumber("fcbcb", 0));
    }
    SmartDashboard.putNumber("vcb", t.getController().getEncoder().getVelocity());
    pid.setReference(SmartDashboard.getNumber("scbcb", 0), CANSparkMax.ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/