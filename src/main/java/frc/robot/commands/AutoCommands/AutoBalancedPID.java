// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.TankDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalancedPID extends PIDCommand {

  TankDrive tankDrive;
  Debouncer debounceSetPoint = new Debouncer(BalanceConstants.debounceTime, DebounceType.kRising);
  String name = "AutoBalancePID: "; 

  public AutoBalancedPID(TankDrive tankDrive) {
    super(
        new PIDController(0.026000, 0.000, 0.0000),
        // This should return the measurement
        () -> tankDrive.getGyro().getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          //tankDrive.autoDrive(-MathUtil.clamp(output, -BalanceConstants.MaxSpeed, BalanceConstants.MaxSpeed), 0);
          SmartDashboard.putNumber("Negative Auto Max Speed", -.7);
          SmartDashboard.putNumber("Positive Auto Max Speed", .7);
          double negativeMax = SmartDashboard.getNumber("Negative Auto Max Speed", -.7);
          double positiveMax = SmartDashboard.getNumber("Positive Auto Max Speed", .7);
          tankDrive.autoDrive(-MathUtil.clamp(output, negativeMax , positiveMax), 0);
        });


        addRequirements(tankDrive);
        this.tankDrive = tankDrive;

        SmartDashboard.putNumber(name + " pvalue", 0.026000);
        SmartDashboard.putNumber(name + " ivalue", 0.000);
        SmartDashboard.putNumber(name + " dvalue", 0.000);

        m_controller.setTolerance(Constants.GyroConstants.balanceRange);
  }
  @Override 
  public void initialize() {
    super.initialize();
    m_controller.setP(SmartDashboard.getNumber(name + " pvalue", 0)); 
    m_controller.setI(SmartDashboard.getNumber(name + " ivalue", 0)); 
    m_controller.setD(SmartDashboard.getNumber(name + " dvalue", 0)); 
    debounceSetPoint.calculate(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debounceSetPoint.calculate(m_controller.atSetpoint());
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getName() + " cancelled");

    super.end(interrupted);
    tankDrive.stopDrive();
  }
  @Override
  public void execute() {
    super.execute();
  }
}
