// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.KeyValue;


/** Add your docs here. */
public class PersistenceData  {

    KeyValue[] list = Constants.PeristentMemory.list;

    public PersistenceData(){
        setup();
    }

    public void setup(){
        DataLogManager.start();

        if (!Preferences.containsKey("NumberOfPowerCycles")){
            Preferences.initInt("NumberOfPowerCycles", 0);
        } 

        for (KeyValue kv : list) {
            System.out.println("Setting up: " + kv.key);
            if (!Preferences.containsKey(kv.key)){
                if (kv.value instanceof Double) {
                    Preferences.initDouble(kv.key, (double) kv.value);
                } else if (kv.value instanceof Integer) {
                    Preferences.initInt(kv.key, (int) kv.value);
                } else if (kv.value instanceof Float) {
                    Preferences.initFloat(kv.key, (float) kv.value);
                } else if (kv.value instanceof Boolean) {
                    Preferences.initBoolean(kv.key, (boolean) kv.value);                    
                } else if (kv.value instanceof String) {
                    Preferences.initString(kv.key, (String) kv.value);  
                } else {
                    System.out.println("failed to insert key:" + kv.key);
                }
            } else {
                //if key exists 
                DataLogManager.log("Key: " + kv.key + " value: " + kv.value);
            } 
        }

        System.out.println("NumberOfPowerCycles " + Preferences.getInt("NumberOfPowerCycles", -1));
        Preferences.setInt("NumberOfPowerCycles", Preferences.getInt("NumberOfPowerCycles", -1)+1);
    }
}

   