// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.TankDrive;

public class AutoBalancePIDv2 extends CommandBase {
  /** Creates a new AutoBalancePIDv2. */
  TankDrive tankDrive;
  Debouncer debounceSetPoint = new Debouncer(BalanceConstants.debounceTime, DebounceType.kRising);
  String name = "AutoBalancePIDv2: "; 
  PIDController pidController = new PIDController(0, 0, 0);
  PIDController pidControllerV = new PIDController(0, 0, 0);

  LinearFilter filter = LinearFilter.movingAverage(10);

  double[] gyroRate = {0,0,0};

  public AutoBalancePIDv2(TankDrive tankDrive) {
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        SmartDashboard.putNumber(name + " pvalue", 0.012000);
        SmartDashboard.putNumber(name + " ivalue", 0.000);
        SmartDashboard.putNumber(name + " dvalue", 0.000);
          // Use addRequirements() here to declare subsystem dependencies.
          SmartDashboard.putNumber(name + "V pvalue", 0.00100);
          SmartDashboard.putNumber(name + "V ivalue", 0.000);
          SmartDashboard.putNumber(name + "V dvalue", 0.000);

        pidController.setTolerance(Constants.GyroConstants.balanceRange);
      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {
     pidController.setP(SmartDashboard.getNumber(name + " pvalue", 0)); 
  pidController.setI(SmartDashboard.getNumber(name + " ivalue", 0)); 
  pidController.setD(SmartDashboard.getNumber(name + " dvalue", 0)); 
  debounceSetPoint.calculate(false);
  pidController.reset();
  pidController.setSetpoint(0);
  
  pidControllerV.setP(SmartDashboard.getNumber(name + "V pvalue", 0)); 
  pidControllerV.setI(SmartDashboard.getNumber(name + "V ivalue", 0)); 
  pidControllerV.setD(SmartDashboard.getNumber(name + "V dvalue", 0)); 
  pidControllerV.reset();
  pidControllerV.setSetpoint(0);
  gyroRate = new double[] {0,0,0};
  filter.reset();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tankDrive.getGyro().getRawGyro(gyroRate);
    double pidPosition = -MathUtil.clamp(pidController.calculate(tankDrive.getGyro().getPitch()), -1 , 1);
    double pidVelosity = -MathUtil.clamp(pidControllerV.calculate(filter.calculate(gyroRate[0])), -1 , 1);
    tankDrive.autoDrive(MathUtil.clamp(pidPosition + pidVelosity, -0.7 , 0.7), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { tankDrive.stopDrive();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debounceSetPoint.calculate(pidController.atSetpoint());
  }
}
