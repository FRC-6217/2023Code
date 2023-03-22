// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;

/** Add your docs here. */
public class ArmPositions {

    private Arm littleArm;
    private BigArm bigArm;

    public ArmPositions( Arm littArm, BigArm bigArm) {
        this.bigArm = bigArm;
        this.littleArm = littArm;
        
    }

    public ParallelCommandGroup ArmsToHighCubeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.highCube, bigArm, bigArm.setpoints.highCube));
        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToHighConeDrop(){
        return ArmsToHighConeDrop(0);
    }
    
    public ParallelCommandGroup ArmsToHighConeDrop(double adding){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
       
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.highCone+adding, bigArm, bigArm.setpoints.highCone));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToMidCubeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.midCube, bigArm, bigArm.setpoints.midCube));
        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToMidConeDrop(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.midCone, bigArm, bigArm.setpoints.midCone));
        return pCommandGroup;
    }

    public SequentialCommandGroup ArmsToFrontPickUp(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        
        pCommandGroup.addCommands(new ArmGoToAngle(littleArm, littleArm.setpoints.lowCube).andThen(new ArmGoToAngle(bigArm, bigArm.setpoints.lowCube)));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToBackPickUp(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
      
        //pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.BacksidePickUpSetPoint(), bigArm, bigArm.setpoints.BacksidePickUpSetPoint()));

        return pCommandGroup;
    }

    public ParallelCommandGroup ArmsToSubStationPickUp(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
       
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.substation, bigArm, bigArm.setpoints.substation));

        return pCommandGroup;
    }

    
    public SequentialCommandGroup ArmsToSaftey(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      
        pCommandGroup.addCommands(new TwoArmsToTwoAngle(littleArm, littleArm.setpoints.saftey, bigArm, bigArm.setpoints.saftey));

        return pCommandGroup;
    }
}
