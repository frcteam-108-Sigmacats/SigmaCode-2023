// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSetUp extends SubsystemBase {
  DigitalInput clawSensor = new DigitalInput(0);
  public Spark blinkin = new Spark(0);
  /** Creates a new LEDSetUp. */
  public LEDSetUp() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void LEDDrive(){
    if(clawSensor.get() == true){
      blinkin.set(0.77);
    }
    else{
      blinkin.set(0.61);
    }
  }
}
