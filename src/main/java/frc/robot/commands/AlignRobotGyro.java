// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignRobotGyro extends CommandBase {
  private SwerveSubsystem swerve;
  private Translation2d translation= new Translation2d(0, 0);
  private double yaw, yawDeg;
  private double desiredAngle = 3.142;
  /** Creates a new AlignRobotGyro. */
  public AlignRobotGyro(SwerveSubsystem swerveSub) {
    swerve = swerveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yaw = swerve.getHeading().getRadians();
    yawDeg = swerve.getHeading().getDegrees();
    if(yawDeg >= 0){
      swerve.drive(translation, -swerve.alignmentController.calculate(yaw, desiredAngle), false);
    }
    else{
      swerve.drive(translation, swerve.alignmentController.calculate(yaw, -desiredAngle), false);
    }
    System.out.println("PID: " + -swerve.alignmentController.calculate(yaw, desiredAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((yawDeg >= 0 && yaw >= desiredAngle) || (yawDeg <= 0 && yaw <= -desiredAngle)){
      return true;
    }
    else{
      return false;
    }
  }
}
