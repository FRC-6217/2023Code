// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSystemConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystem.ARM_SELECTION;

public class ArmGoToAngle extends CommandBase {
  /** Creates a new ArmPIDTuner. */
  ARM_SELECTION selection;
  ArmSystem armSystem;
  boolean isTuning;
  double setPoint;

  PIDController pidController = new PIDController(0, 0, 0);
  CANSparkMax arm;

  String pKey = " p: ";
  String iKey = " i: ";
  String dKey = " d: ";
  String setPointKey = " setpoint: ";
  String bigArmKey = "big arm";
  String littleArmKey = "little arm";

  public ArmGoToAngle(ArmSystem armSystem, ARM_SELECTION selection, double setPoint, boolean isTuning) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSystem);
    this.armSystem = armSystem;
    this.selection = selection;
    this.isTuning = isTuning;
    this.setPoint = setPoint;

    switch (selection) {
      case LITTLE_ARM:
        arm = armSystem.getLittleArmController();
        break;
      case BIG_ARM:
        arm = armSystem.getBigArmController();
        break;
    }

    SmartDashboard.putNumber(bigArmKey + pKey, ArmSystemConstants.BigArmAngle.Pvalue);
    SmartDashboard.putNumber(bigArmKey + iKey, ArmSystemConstants.BigArmAngle.Ivalue);
    SmartDashboard.putNumber(bigArmKey + dKey, ArmSystemConstants.BigArmAngle.Dvalue);
    SmartDashboard.putNumber(bigArmKey + setPointKey, 0);

    SmartDashboard.putNumber(littleArmKey + pKey, ArmSystemConstants.LittleArmAngle.Pvalue);
    SmartDashboard.putNumber(littleArmKey + iKey, ArmSystemConstants.LittleArmAngle.Ivalue);
    SmartDashboard.putNumber(littleArmKey + dKey, ArmSystemConstants.LittleArmAngle.Dvalue);
    SmartDashboard.putNumber(littleArmKey + setPointKey, 0);

  }

  public ArmGoToAngle(ArmSystem armSystem, ARM_SELECTION selection, double setPoint) {
    this(armSystem, selection, setPoint, false);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (selection) {
      case BIG_ARM:
        pidController.setP(SmartDashboard.getNumber(bigArmKey + pKey, ArmSystemConstants.BigArmAngle.Pvalue));
        pidController.setI(SmartDashboard.getNumber(bigArmKey + iKey, ArmSystemConstants.BigArmAngle.Ivalue));
        pidController.setD(SmartDashboard.getNumber(bigArmKey + dKey, ArmSystemConstants.BigArmAngle.Dvalue));
        if (isTuning) {
          pidController.setSetpoint(SmartDashboard.getNumber(bigArmKey + setPointKey, 0));
        }else {
          pidController.setSetpoint(setPoint);

        }
        break;
      case LITTLE_ARM:
        pidController.setP(SmartDashboard.getNumber(littleArmKey + pKey, ArmSystemConstants.LittleArmAngle.Pvalue));
        pidController.setI(SmartDashboard.getNumber(littleArmKey + iKey, ArmSystemConstants.LittleArmAngle.Ivalue));
        pidController.setD(SmartDashboard.getNumber(littleArmKey + dKey, ArmSystemConstants.LittleArmAngle.Dvalue));
        if (isTuning) {
          pidController.setSetpoint(SmartDashboard.getNumber(littleArmKey + setPointKey, 0));
        } else {
          pidController.setSetpoint(setPoint);

        }
        break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    switch (selection) {
      case BIG_ARM:
        speed = pidController.calculate(armSystem.getBigArmPosition());
      case LITTLE_ARM:
        speed = pidController.calculate(armSystem.getLittleArmPositon());
    }

    arm.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
