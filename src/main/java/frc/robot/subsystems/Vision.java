// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limeLight.getEntry("ty");
    NetworkTableEntry ty = limeLight.getEntry("tx");
    NetworkTableEntry ta = limeLight.getEntry("ta");
  /** Creates a new Vision. */
  public Vision() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
