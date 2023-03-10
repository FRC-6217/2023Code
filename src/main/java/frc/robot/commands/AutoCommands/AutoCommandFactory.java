// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IArmConstants;
import frc.robot.commands.ArmCommands.TwoArmsToTwoAngle;
import frc.robot.commands.AutoCommands.DriveToBalanced.DirectionB;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced.Direction;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;

/** Add your docs here. */
public class AutoCommandFactory {
    RobotContainer robotContainer;
    Arm littleArm;
    BigArm bigArm;
    public AutoCommandFactory(RobotContainer robotContainer, Arm littArm, BigArm bigArm){
        this.robotContainer = robotContainer;
        this.bigArm = bigArm;
        this.littleArm = littArm;
    }

    // public SequentialCommandGroup getAutoDropOffHigh(){
    //     SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    //     commandGroup.addCommands(Commands.waitSeconds(.5));
        
    //     commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -40));
    //     commandGroup.addCommands(new BigArmToLimitSwitch(armSystem));
    //     commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -157));
    //     commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, 24, .45));
    //     commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    //     commandGroup.addCommands(Commands.waitSeconds(.5));
    //     commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -24, 0.45));
    //     commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.BIG_ARM, 10));
    //     commandGroup.addCommands(new LittleArmToLimitSwitch(armSystem));
    //     System.out.println("Done With Little");
      
    //     return commandGroup;
    //   }


    /*public SequentialCommandGroup getMiddleLeaveBalance(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(new PrintCommand("Driving off1"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.backwards));
    commandGroup.addCommands(new PrintCommand("Driving off2"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off3"));
    commandGroup.addCommands(new DriveToBalanced(mTankDrive, DirectionB.forwards));
    commandGroup.addCommands(new PrintCommand("Driving off4"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off5"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.forwards));
    commandGroup.addCommands(new PrintCommand("Driving off6"));
    commandGroup.addCommands(new AutoBalancedPID(mTankDrive));
    return commandGroup;
  } */


    public ParallelCommandGroup ArmsToHighCubeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getHighCubeSetPoint(), bigArm, bigArm.getConstants().getHighCubeSetPoint()));
        return pCommandGroup;
    }

    
    public ParallelCommandGroup ArmsToHighConeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
       
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getHighConeSetPoint(), bigArm, bigArm.getConstants().getHighConeSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToMidCubeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
       
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getMidCubeSetPoint(), bigArm, bigArm.getConstants().getMidCubeSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToMidConeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
      
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getMidConeSetPoint(), bigArm, bigArm.getConstants().getMidConeSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToFrontPickUp(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getPickUpConeSetPoint(), bigArm, bigArm.getConstants().getPickUpConeSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToBackPickUp(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
      
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getBacksidePickUpSetPoint(), bigArm, bigArm.getConstants().getBacksidePickUpSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToSubStationPickUp(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
       
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getSubstationSetPoint(), bigArm, bigArm.getConstants().getSubstationSetPoint()));

        return pCommandGroup;
    }

    public SequentialCommandGroup DriveOverChargeStation(){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(robotContainer.mTankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(robotContainer.mTankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveToBalanced(robotContainer.mTankDrive, DirectionB.forwards));
        commandGroup.addCommands(new DriveToDistanceInches(robotContainer.mTankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveUntilUnBalanced(robotContainer.mTankDrive, Direction.forwards));
        commandGroup.addCommands(new AutoBalancedPID(robotContainer.mTankDrive));
        return commandGroup;
    }

    public SequentialCommandGroup ArmsToSaftey(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.getConstants().getSafteySetPoint(), bigArm, bigArm.getConstants().getSafteySetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup DriveOverChargeStationArmsToSaftey(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        if (robotContainer.bigArm.getAngle() > 90) {
            //add new command to avoid 
        }
        pCommandGroup.addCommands(DriveOverChargeStation());
        pCommandGroup.addCommands(ArmsToSaftey());
        return pCommandGroup;
    }

    public ParallelCommandGroup DriveOverChargeStationArmsFlipOver(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStation());
 

        return pCommandGroup;
    }



    public SequentialCommandGroup MoveWithSaftey(double bigSetpoint, double littleSetpoint){
        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();
        return sCommandGroup;
    }
}
