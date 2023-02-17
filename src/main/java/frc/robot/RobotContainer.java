// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalancedPID;
import frc.robot.commands.Autos;
import frc.robot.commands.CancelDriveTrain;
import frc.robot.commands.Drivetounbalence;
import frc.robot.commands.FindKs;
import frc.robot.commands.FindKv;
import frc.robot.commands.PersistenceData;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopDrivePID;
import frc.robot.commands.AutoCommands.AngleDrive;
import frc.robot.commands.AutoCommands.InchesDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.PIDDriveTrain;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  public final TankDrive mTankDrive = new TankDrive();
  //public final PIDDriveTrain mTankDrive = new PIDDriveTrain();
  
  //private final InchesDrive inchesDrive12forward = new InchesDrive(mTankDrive, 12, .3);
 // private final InchesDrive inchesDrive12back = new InchesDrive(mTankDrive, -12, .3);

  //private final SimpleMotorController bigArm = new SimpleMotorController(10, "BigArm");
  //private final SimpleMotorController littleArm = new SimpleMotorController(11, "LittleArm");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kXboxDriver);
  private final CommandJoystick driveJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final PDP pdp = new PDP();
  //private final TestCoolBeans t = new TestCoolBeans();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println("RUNNING ROBOT: " + Constants.uniqueRobotConstants.getName());
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
    //driveJoystick.button(1).onTrue(new FindKv(mTankDrive));
   // driveJoystick.button(1).onTrue(new SequentialCommandGroup(new InchesDrive(mTankDrive, 12, 0.3), new InchesDrive(mTankDrive, -12, 0.3)));
   // driveJoystick.button(2).onTrue(new GoToAngle(mTankDrive, -360, .36));

   driveJoystick.button(12).onTrue(Commands.runOnce(mTankDrive::toggleturning, mTankDrive));
    driveJoystick.button(OperatorConstants.toggleBreak2).onTrue(Commands.runOnce(mTankDrive::toggleBreaks, mTankDrive));
    //mTankDrive.setDefaultCommand( new TeleopDrivePID(mTankDrive, driveJoystick));
    mTankDrive.setDefaultCommand( new TeleopDrive(mTankDrive, driveJoystick));
    driveJoystick.button(4).onTrue(Commands.runOnce(mTankDrive::resetPosition, mTankDrive));
    driveJoystick.button(11).onTrue(new CancelDriveTrain(mTankDrive));
    driveJoystick.button(3).whileTrue(new AutoBalancedPID(mTankDrive).andThen(Commands.print("hey")));
    driveJoystick.button(6).onTrue(new Drivetounbalence(mTankDrive).andThen(new AngleDrive(mTankDrive)));
    driveJoystick.button(7).onTrue(new InchesDrive( mTankDrive,-20,.4));
    /*driveJoystick.button(OperatorConstants.littleArmFoward).onTrue(Commands.runOnce(littleArm::on, littleArm)).onFalse(Commands.runOnce(littleArm::off, littleArm));
    driveJoystick.button(OperatorConstants.bigArmForward).onTrue(Commands.runOnce(bigArm::on, bigArm)).onFalse(Commands.runOnce(bigArm::off, bigArm));
    driveJoystick.button(OperatorConstants.littleArmBack).onTrue(Commands.runOnce(littleArm::reverse, littleArm)).onFalse(Commands.runOnce(littleArm::off, littleArm));
    driveJoystick.button(OperatorConstants.bigArmBack).onTrue(Commands.runOnce(bigArm::reverse, bigArm)).onFalse(Commands.runOnce(bigArm::off, bigArm));*/
    //CommandScheduler.getInstance().setDefaultCommand(mTankDrive, new TeleopDrive(mTankDrive, driveJoystick));
    //driveJoystick.button(OperatorConstants.stayPutCommandButtonbottonunsed12).onTrue(new FindKs(mTankDrive));
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
}
