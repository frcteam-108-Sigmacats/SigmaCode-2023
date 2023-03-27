// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  private SwerveSubsystem swerveMech;
  private Pose2d blueMinPos = new Pose2d(3.44, 0, null);
  private Pose2d blueMaxPos = new Pose2d(4.33, 0, null);
  private Pose2d redMinPos = new Pose2d(13.13, 0, null);
  private Pose2d redMaxPos = new Pose2d(12.18, 0, null);
  private Pose2d minPos = new Pose2d();
  private Pose2d maxPos = new Pose2d();
  private Translation2d translation;
  private double ySpeed;
  private double offset = 0;
  public AutoBalance(SwerveSubsystem swerveSub) {
    swerveMech = swerveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      minPos = blueMinPos;
      maxPos = blueMaxPos;
    }
    else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      minPos = redMinPos;
      maxPos = redMaxPos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(swerveMech.getPitch().getRadians() >= offset && (swerveMech.getPose().getX() > minPos.getX() && swerveMech.getPose().getX() < maxPos.getX())){
      ySpeed = Math.sin(swerveMech.getPitch().getRadians()) * 2;
    }
    else{
      ySpeed = 0;
    }
    translation = new Translation2d(ySpeed, 0);
    swerveMech.drive(translation, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ySpeed == 0 || swerveMech.getPitch().getRadians() == offset){
      return true;
    }
    return false;
  }
}
