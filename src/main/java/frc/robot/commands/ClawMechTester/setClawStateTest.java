// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawMechTester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawMechanism;

public class setClawStateTest extends CommandBase {
  private ClawMechanism clawMech;
  private double armPos;
  private double clawPos;
  /** Creates a new setClawStateTest. */
  public setClawStateTest(ClawMechanism clawSub, double armPos, double clawPos) {
    clawMech = clawSub;
    this.armPos = armPos;
    this.clawPos = clawPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawMech.setArmPosition(armPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
