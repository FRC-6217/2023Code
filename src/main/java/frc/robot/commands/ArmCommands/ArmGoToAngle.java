// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem.Arm;

public class ArmGoToAngle extends CommandBase {
  Arm arm;
  boolean isTuning;
  double setPoint;

  boolean isError;

  PIDController pidController = new PIDController(0, 0, 0);

  String pKey = " p: ";
  String iKey = " i: ";
  String dKey = " d: ";
  String setPointKey = " setpoint: ";

  public ArmGoToAngle(Arm arm, double setPoint, boolean isTuning) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.isTuning = isTuning;
    this.setPoint = setPoint;
    isError = false;

    SmartDashboard.putNumber(arm.getArmName() + pKey, arm.getConstants().getPIDConstants().p);
    SmartDashboard.putNumber(arm.getArmName() + iKey, arm.getConstants().getPIDConstants().i);
    SmartDashboard.putNumber(arm.getArmName() + dKey, arm.getConstants().getPIDConstants().d);
    SmartDashboard.putNumber(arm.getArmName() + setPointKey, 0);

    pidController.setTolerance(1.5);

  }

  public ArmGoToAngle(Arm armSystem, double setPoint) {
    this(armSystem, setPoint, false);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidController.reset();

    pidController.setP(SmartDashboard.getNumber(arm.getArmName() + pKey, arm.getConstants().getPIDConstants().p));
    pidController.setI(SmartDashboard.getNumber(arm.getArmName() + iKey, arm.getConstants().getPIDConstants().i));
    pidController.setD(SmartDashboard.getNumber(arm.getArmName() + dKey, arm.getConstants().getPIDConstants().d));
    if (isTuning) {
      pidController.setSetpoint(SmartDashboard.getNumber(arm.getArmName() + setPointKey, 0));
    }else {
      pidController.setSetpoint(setPoint);

    }



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (arm.getAngle() > arm.getConstants().getStopThreshold() || arm.getAngle() < -arm.getConstants().getStopThreshold()) {
      System.out.println("Analog Error: " + arm.getName());
      isError = true;
    }

    double speed = MathUtil.clamp(pidController.calculate(arm.getAngle()), -arm.getConstants().getMaxAutoSpeed(), arm.getConstants().getMaxAutoSpeed());
    
    arm.armConstantSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() || isError;
  }
}
