// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PersistenceData;
import frc.robot.commands.StayPut;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TestCommandCoolBeans;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.SimpleMotorController;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TestCoolBeans;
import frc.robot.subsystems.sensor.Gyro;
import frc.robot.subsystems.sensor.Xcelerameter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final PersistenceData mData = new PersistenceData();
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final TankDrive mTankDrive = new TankDrive();

  private final SimpleMotorController bigArm = new SimpleMotorController(10, "BigArm");
  private final SimpleMotorController littleArm = new SimpleMotorController(11, "LittleArm");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kXboxDriver);
  private final CommandJoystick driveJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final Gyro gyro = new Gyro();
  private final PDP pdp = new PDP();
  private final TestCoolBeans t = new TestCoolBeans();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {


    driveJoystick.button(1).onTrue(Commands.runOnce(littleArm::on, littleArm)).onFalse(Commands.runOnce(littleArm::off, littleArm));
    driveJoystick.button(2).onTrue(Commands.runOnce(bigArm::on, bigArm)).onFalse(Commands.runOnce(bigArm::off, bigArm));

    //CommandScheduler.getInstance().setDefaultCommand(mTankDrive, new TeleopDrive(mTankDrive, driveJoystick));
    mTankDrive.setDefaultCommand( new TeleopDrive(mTankDrive, driveJoystick));
/*
    driveJoystick.button(OperatorConstants.stayPutCommandButton).whileTrue(new StayPut(mTankDrive, gyro));
    driveJoystick.button(OperatorConstants.outlevelbutton).whileTrue(new AutoLevel(mTankDrive,gyro));
*/



    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public PersistenceData getmData() {
    return mData;
  }

  public ExampleSubsystem getM_exampleSubsystem() {
    return m_exampleSubsystem;
  }

  public TankDrive getmTankDrive() {
    return mTankDrive;
  }

  public SimpleMotorController getBigArm() {
    return bigArm;
  }

  public CommandJoystick getDriveJoystick() {
    return driveJoystick;
  }

  public Gyro getGyro() {
    return gyro;
  }

  public PDP getPdp() {
    return pdp;
  }

  public TestCoolBeans getT() {
    return t;
  }
}
