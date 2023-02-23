// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SetClawStates extends CommandBase {
  private int clawState;
  private Claw clawMech;
  private int counter;
  /** Creates a new SetClawStates. */
  public SetClawStates(Claw clawSub, int clawState) {
    this.clawState = clawState;
    clawMech = clawSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(clawState == 1){
      if(counter > 300){
        clawMech.setClawStates(clawState);
      }
    }
    else{
      clawMech.setClawStates(clawState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawMech.setClawStates(clawState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
