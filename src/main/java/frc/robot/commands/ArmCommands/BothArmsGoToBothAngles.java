// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSystemConstants;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;


public class BothArmsGoToBothAngles extends CommandBase {

  BigArm bigArm;
  Arm littleArm;
  double bigArmFinalSetPoint;
  double littleArmFinalSetPoint;
  boolean isTuning;

  PIDController bigPid;
  PIDController littlePid;

  final String pKey = " p: ";
  final String iKey = " i: ";
  final String dKey = " d: ";
  final String setPointKey = " setpoint: ";

  double littleCurrentSetPoint;
  double bigCurrentSetPoint;

  boolean isDetouringLowToHigh = false;
  boolean isDetouringHighToLow = false;

  
  public BothArmsGoToBothAngles(BigArm bigArm, Arm littleArm, double bigArmSetPoint, double littleArmSetPoint, boolean isTuning) {
    addRequirements(bigArm, littleArm);
    this.bigArm = bigArm;
    this.littleArm = littleArm;
    this.bigArmFinalSetPoint = bigArmSetPoint;
    this.littleArmFinalSetPoint = littleArmSetPoint;
    this.isTuning = isTuning;


    bigPid = new PIDController(bigArm.getConstants().getPIDConstants().p, bigArm.getConstants().getPIDConstants().i, bigArm.getConstants().getPIDConstants().d);
    littlePid = new PIDController(littleArm.getConstants().getPIDConstants().p, littleArm.getConstants().getPIDConstants().i, littleArm.getConstants().getPIDConstants().d);

    SmartDashboard.putNumber(bigArm.getArmName() + pKey, bigArm.getConstants().getPIDConstants().p);
    SmartDashboard.putNumber(bigArm.getArmName() + iKey, bigArm.getConstants().getPIDConstants().i);
    SmartDashboard.putNumber(bigArm.getArmName() + dKey, bigArm.getConstants().getPIDConstants().d);
    SmartDashboard.putNumber(bigArm.getArmName() + setPointKey, 0);

    SmartDashboard.putNumber(littleArm.getArmName() + pKey, littleArm.getConstants().getPIDConstants().p);
    SmartDashboard.putNumber(littleArm.getArmName() + iKey, littleArm.getConstants().getPIDConstants().i);
    SmartDashboard.putNumber(littleArm.getArmName() + dKey, littleArm.getConstants().getPIDConstants().d);
    SmartDashboard.putNumber(littleArm.getArmName() + setPointKey, 0);
  }

  public BothArmsGoToBothAngles(BigArm bigArm, Arm littleArm, double bigArmSetPoint, double littleArmSetPoint) {
    this(bigArm, littleArm, bigArmSetPoint, littleArmSetPoint, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    bigPid.reset();

    bigPid.setP(SmartDashboard.getNumber(bigArm.getArmName() + pKey, bigArm.getConstants().getPIDConstants().p));
    bigPid.setI(SmartDashboard.getNumber(bigArm.getArmName() + iKey, bigArm.getConstants().getPIDConstants().i));
    bigPid.setD(SmartDashboard.getNumber(bigArm.getArmName() + dKey, bigArm.getConstants().getPIDConstants().d));

    littlePid.reset();

    littlePid.setP(SmartDashboard.getNumber(littleArm.getArmName() + pKey, littleArm.getConstants().getPIDConstants().p));
    littlePid.setI(SmartDashboard.getNumber(littleArm.getArmName() + iKey, littleArm.getConstants().getPIDConstants().i));
    littlePid.setD(SmartDashboard.getNumber(littleArm.getArmName() + dKey, littleArm.getConstants().getPIDConstants().d));

    isDetouringLowToHigh = isCrossingNoNoZoneFromLow();
    isDetouringHighToLow = isCrossingNoNoZoneFromHigh();


    if (isTuning) {
      // don't worry about being too tall in tuning mode
      bigPid.setSetpoint(SmartDashboard.getNumber(bigArm.getArmName() + setPointKey, 0));
      littlePid.setSetpoint(SmartDashboard.getNumber(littleArm.getArmName() + setPointKey, 0));
    } else if (isDetouringHighToLow ) {
      // first we need to lower the little arm to a safe position before moving big arm
      littlePid.setSetpoint(ArmSystemConstants.littleArmDetourAngleHighToLow);
    } else if (isDetouringLowToHigh) {
      // first we need to lower the little arm to a safe position before moving big arm
      littlePid.setSetpoint(ArmSystemConstants.littleArmDetourAngleLowToHigh);  
    } else {
      bigPid.setSetpoint(bigArmFinalSetPoint);
      littlePid.setSetpoint(littleArmFinalSetPoint);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double littleArmSpeed = 0;
    double bigArmSpeed = 0;

    if (isDetouringLowToHigh || isDetouringHighToLow) {
      // if we are detouring only move little arm
      littleArmSpeed = MathUtil.clamp(littlePid.calculate(littleArm.getAngle()), -littleArm.getConstants().getMaxAutoSpeed(), littleArm.getConstants().getMaxAutoSpeed());
      bigArmSpeed = 0;

      if (littlePid.atSetpoint()) {
        // if we are finished with the detour, we can now proceed to our actual setpoint
        littlePid.reset();
        littlePid.setSetpoint(littleArmFinalSetPoint);
        isDetouringHighToLow = false;
        isDetouringLowToHigh = false;
      }

    } else {
      littleArmSpeed = MathUtil.clamp(littlePid.calculate(littleArm.getAngle()), -littleArm.getConstants().getMaxAutoSpeed(), littleArm.getConstants().getMaxAutoSpeed());
      bigArmSpeed = MathUtil.clamp(bigPid.calculate(bigArm.getAngle()), -bigArm.getConstants().getMaxAutoSpeed(), bigArm.getConstants().getMaxAutoSpeed());

    }
  
    littleArm.armConstantSpeed(littleArmSpeed);
    bigArm.armConstantSpeed(bigArmSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bigArm.stop();
    littleArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bigPid.atSetpoint() && littlePid.atSetpoint();
  }

  public boolean isAngleWithinZone(double angle, double[] range) {
    return range[0] > angle && range[1] < angle;
  }

  private boolean isCrossingNoNoZoneFromHigh() {
    return (bigArmFinalSetPoint > ArmSystemConstants.bigArmDetourRequiredZone[1]) && (bigArm.getAngle() < ArmSystemConstants.bigArmDetourRequiredZone[1]);
  }

  private boolean isCrossingNoNoZoneFromLow() {
    return (bigArmFinalSetPoint < ArmSystemConstants.bigArmDetourRequiredZone[0]) && (bigArm.getAngle() > ArmSystemConstants.bigArmDetourRequiredZone[0]);
  }

}
