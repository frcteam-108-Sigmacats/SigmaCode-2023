// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetUpMotors;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Claw extends SubsystemBase {
  public static CANSparkMax intakeMotor = new CANSparkMax(SetUpMotors.clawIntake, MotorType.kBrushless);

  public DoubleSolenoid firstCylinder = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
  /** Creates a new Claw. */
  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
