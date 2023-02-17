// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawMechSetUp;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Claw extends SubsystemBase {
  public static CANSparkMax intakeMotor = new CANSparkMax(ClawMechSetUp.clawIntake, MotorType.kBrushless);
  public boolean gamePiece, isCone, isCube;
  public static DigitalInput clawSensor = new DigitalInput(0);
  public static DoubleSolenoid firstCylinder = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
  /** Creates a new Claw. */
  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*public void intakeGround(double speed, boolean extend, boolean gamePiece){
    if (extend && gamePiece){
      firstCylinder.set(Value.kForward);
      intakeMotor.set(speed);
    }
    else if (extend && !gamePiece){
      firstCylinder.set(Value.kForward);
      intakeMotor.set(-speed);
    }
  }*/
  public void intakeStates(int clawStates, double speed){
    switch (clawStates){

      case 0: 
        intakeMotor.set(0);
        firstCylinder.set(Value.kOff);
        break;

      case 1:
        if(clawSensor.get() == true){
           gamePiece = true;
           isCone = true;
           isCube = false;

        }
        else{
          gamePiece = false;
        }

        if (gamePiece == true && isCone == true){
          intakeMotor.set(0);
          firstCylinder.set(Value.kReverse);
        }
        else{
          intakeMotor.set(speed);
          firstCylinder.set(Value.kForward);
        }
        break;

      case 2:
        if(clawSensor.get() == true){
          gamePiece = true;
          isCone = false;
          isCube = true;

        }
        else{
          gamePiece = false;
        }

        if (gamePiece == true && isCube == true){
          intakeMotor.set(0);
          firstCylinder.set(Value.kReverse);
        }
        else{
          intakeMotor.set(-speed);
          firstCylinder.set(Value.kForward);
        }

      case 3:
        if(clawSensor.get() == true){
          gamePiece = true;
          isCone = true;
          isCube = false;

        }
        else{
          gamePiece = false;
        }

        if (gamePiece == true && isCone == true){
          intakeMotor.set(0);
        }
        else{
          intakeMotor.set(speed);
        }
        break;

      case 4:
        if(clawSensor.get() == true){
          gamePiece = true;
          isCone = false;
          isCube = true;

        }
        else{
          gamePiece = false;
        }

        if (gamePiece == true && isCube == true){
          intakeMotor.set(0);
        }
        else{
          intakeMotor.set(-speed);
        }
        
    }
  }
}
