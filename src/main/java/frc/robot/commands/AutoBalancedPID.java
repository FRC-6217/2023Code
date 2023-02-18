// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  public AutoBalancedPID(TankDrive tankDrive) {
    super(
        new PIDController(0.033000, 0.0010, 0.003000),
        // This should return the measurement
        () -> tankDrive.getGyro().getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
        tankDrive.autoDrive(-output, 0);
        });


        addRequirements(tankDrive);
        this.tankDrive = tankDrive;

        SmartDashboard.putNumber(" pvalue", 0.033000);
        SmartDashboard.putNumber(" ivalue", 0.0010);
        SmartDashboard.putNumber(" dvalue", 0.003000);

        m_controller.setTolerance(Constants.GyroConstants.balanceRange);
  }
  @Override 
  public void initialize() {
    super.initialize();
    m_controller.setP(SmartDashboard.getNumber(" pvalue", 0)); 
    m_controller.setI(SmartDashboard.getNumber(" ivalue", 0)); 
    m_controller.setD(SmartDashboard.getNumber(" dvalue", 0)); 
    debounceSetPoint.calculate(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return debounceSetPoint.calculate(m_controller.atSetpoint());
  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    tankDrive.autoDrive(0, 0);
  }
  @Override
  public void execute() {
    super.execute();
  }
}
