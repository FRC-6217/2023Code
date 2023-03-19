// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;

public class TwoArmsToTwoAngle extends CommandBase{
  /** Creates a new TwoArmsToTwoAngle. */
  Arm littleArm;
  BigArm bigArm;
  double littleSetpoint;
  double bigSetpoint;


  PIDController bigPidController = new PIDController(0, 0, 0);
  PIDController littlePidController = new PIDController(0, 0, 0);

  SlewRateLimiter bigArmSlewRate;
  SlewRateLimiter littleArmSlewRate;

  String pKey = " p: ";
  String iKey = " i: ";
  String dKey = " d: ";
  String setPointKey = " setpoint: ";

  public TwoArmsToTwoAngle(Arm littleArm, double littleSetpoint, BigArm bigArm, double bigSetpoint) {
   //addRequirements(littleArm, bigArm);
   this.bigArm = bigArm;
   this.littleArm = littleArm;
   this.littleSetpoint = littleSetpoint;
   this.bigSetpoint = bigSetpoint;
  

    SmartDashboard.putNumber("LittleArm" + pKey, littleArm.getConstants().getPIDConstants().p);
    SmartDashboard.putNumber("LittleArm" + iKey, littleArm.getConstants().getPIDConstants().i);
    SmartDashboard.putNumber("LittleArm" + dKey, littleArm.getConstants().getPIDConstants().d);
    SmartDashboard.putNumber("LittleArm" + setPointKey, 0);
  
    SmartDashboard.putNumber("BigArm" + pKey, bigArm.getConstants().getPIDConstants().p);
    SmartDashboard.putNumber("BigArm" + iKey, bigArm.getConstants().getPIDConstants().i);
    SmartDashboard.putNumber("BigArm" + dKey, bigArm.getConstants().getPIDConstants().d);
    SmartDashboard.putNumber("BigArm" + setPointKey, 0);
  }

  // Called when the command is initially scheduled.

  public void initialize() {
    bigPidController.reset();
    littlePidController.reset();

    littlePidController.setP(SmartDashboard.getNumber("LittleArm" + pKey, littleArm.getConstants().getPIDConstants().p));
    littlePidController.setI(SmartDashboard.getNumber("LittleArm" + iKey, littleArm.getConstants().getPIDConstants().i));
    littlePidController.setD(SmartDashboard.getNumber("LittleArm" + dKey, littleArm.getConstants().getPIDConstants().d));
  
    bigPidController.setP(SmartDashboard.getNumber("BigArm" + pKey, bigArm.getConstants().getPIDConstants().p));
    bigPidController.setI(SmartDashboard.getNumber("BigArm" + iKey, bigArm.getConstants().getPIDConstants().i));
    bigPidController.setD(SmartDashboard.getNumber("BigArm" + dKey, bigArm.getConstants().getPIDConstants().d));

    SmartDashboard.putNumber("LittleArm SetPoint", littleSetpoint);
    SmartDashboard.putNumber("BigArm SetPoint", bigSetpoint);

    littlePidController.setSetpoint(littleSetpoint);
    bigPidController.setSetpoint(bigSetpoint);

    bigArmSlewRate = new SlewRateLimiter(SmartDashboard.getNumber("big arm slew rate: ", 1));
    littleArmSlewRate = new SlewRateLimiter(SmartDashboard.getNumber("little arm slew rate: ", 1));


    //todo move to constants
    littlePidController.setTolerance(1.5);
    bigPidController.setTolerance(1.5);
  
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {

    if(bigPidController.atSetpoint()){
      bigArm.stop();
    }else{

    double pidoutput = bigPidController.calculate(bigArm.getAngle());
    double bigSpeed = MathUtil.clamp(pidoutput, -bigArm.getConstants().getMaxAutoSpeed(), bigArm.getConstants().getMaxAutoSpeed());
  //  double slewRatedOutput = bigArmSlewRate.calculate(bigSpeed);


    bigArm.armConstantSpeed(bigSpeed);
   }

   if(littlePidController.atSetpoint()){
    littleArm.stop();
  }else{
    double littleSpeed = MathUtil.clamp(littlePidController.calculate(littleArm.getAngle()), -littleArm.getConstants().getMaxAutoSpeed(), littleArm.getConstants().getMaxAutoSpeed());
   // double slewRatedOutput = littleArmSlewRate.calculate(littleSpeed);
    littleArm.armConstantSpeed(littleSpeed);

 }
  }

  // Called once the command ends or is interrupted.

  public void end(boolean interrupted) {
    bigArm.stop();
    littleArm.stop();
    System.out.println("Arm Command Finished b: " + bigSetpoint + " l: " + littleSetpoint);
  }

  // Returns true when the command should end.

  public boolean isFinished() {
   
    return bigPidController.atSetpoint() && littlePidController.atSetpoint();
  }
}
