// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;

public class BottomIntake extends CommandBase {
  private Claw claw = RobotContainer.m_Claw;
  private double speed;
  private int counter;
  private boolean isReturn;
  /** Creates a new BottomIntake. */
  public BottomIntake(Claw claw, double speed, boolean isReturn) {
    this.isReturn = isReturn;
    this.claw = claw;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isReturn == true){
      claw.bottomIntake(speed, counter);
    }
    else{
      claw.returnBottomIntake(speed, counter);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
