// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class Vision extends SubsystemBase {
  public double xVal, yVal, area, skew;
  public boolean isCone, isCube;
  private double rotationSpeed;
  private double yOffset;
  private SwerveSubsystem swerve;
  private PIDController turn = new PIDController(0.03, 0, 12);
  private SlewRateLimiter rotationLimit = new SlewRateLimiter(0.5);
  private Translation2d translation = new Translation2d(0, 0);
  private boolean fieldRelative;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //LimeLight is flipped over so my x is now my y and my y is now my x.
  NetworkTableEntry tx = table.getEntry("ty");
  NetworkTableEntry ty = table.getEntry("tx");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ts = table.getEntry("ts");
  
  /** Creates a new Vision. */
  public Vision() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateValues(){
    xVal = tx.getDouble(0.0);
    yVal = ty.getDouble(0.0);
  }

  public void switchingPipeline(){
    if(Claw.isCone == true){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
    else if(Claw.isCube == true)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  }

  public void reflectiveAlign(){
    yOffset = 0;
    rotationSpeed = rotationLimit.calculate(turn.calculate(xVal, yOffset));

    swerve.drive(translation, rotationSpeed, fieldRelative);
  }

  public void aprilTagAlign(){
    
  }
}
