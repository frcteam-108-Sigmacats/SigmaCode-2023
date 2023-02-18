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
  public static CANSparkMax intakeMotor;
  public boolean gamePiece, isCone, isCube;
  private double coneOuttakeSpd, cubeOuttakeSpd;
  public static DigitalInput clawSensor;
  public static DoubleSolenoid clawExtenders;
  /** Creates a new Claw. */
  public Claw() {
    intakeMotor = new CANSparkMax(ClawMechSetUp.clawIntake, MotorType.kBrushless);
    coneOuttakeSpd = -0.75;
    cubeOuttakeSpd = 0.5;
    clawSensor = new DigitalInput(0);
    clawExtenders = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
  }

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
        clawExtenders.set(Value.kOff);
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
          clawExtenders.set(Value.kReverse);
        }
        else{
          intakeMotor.set(speed);
          clawExtenders.set(Value.kForward);
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
          clawExtenders.set(Value.kReverse);
        }
        else{
          intakeMotor.set(-speed);
          clawExtenders.set(Value.kForward);
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
  public void setClawStates(int clawStates){
    switch(clawStates){
      //Starting config state
      case 0:
      //Set Arm Pos
        intakeMotor.set(0);
        clawExtenders.set(Value.kReverse);
        clawExtenders.set(Value.kOff);
      //Drive config state
      case 1:
        if(gamePiece){//and not in community zone
          //set arm position to drive state
          intakeMotor.set(0);
          clawExtenders.set(Value.kReverse);
          clawExtenders.set(Value.kOff);
        }
        else{
          clawStates = 0;
        }
      //Set high arm pos used for auto claw
      case 2:
        if(gamePiece){//and in the community zone
          //set arm position to high peg state
          intakeMotor.set(0);
          clawExtenders.set(Value.kReverse);
          clawExtenders.set(Value.kOff);
        }
      //Set mid arm pos
      case 3:
        if(gamePiece){
          //set mid arm pos
          intakeMotor.set(0);
          clawExtenders.set(Value.kReverse);
          clawExtenders.set(Value.kOff);
        }
        else{
          clawStates = 0;
        }
      //Set low arm pos
      case 4:
        if(gamePiece){
          //set low arm pos
          intakeMotor.set(0);
          clawExtenders.set(Value.kReverse);
          clawExtenders.set(Value.kOff);
        }
    }
  }
  public void setOuttakeSpd(){
    if(isCone == true && isCube == false){
      intakeMotor.set(coneOuttakeSpd);
    }
    else if(isCone == false && isCube == true){
      intakeMotor.set(cubeOuttakeSpd);
    }
  }
}
