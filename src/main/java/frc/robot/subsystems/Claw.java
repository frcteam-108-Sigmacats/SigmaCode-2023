// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawMechSetUp;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Claw extends SubsystemBase {
  public CANSparkMax leftClawArmMotor, rightClawArmMotor;
  public SparkMaxPIDController rotateArmPID;
  public AbsoluteEncoder throughBoreAbs;
  public static CANSparkMax clawIntake;
  public static boolean gamePiece;
  public static boolean isCone;
  public static boolean isCube;
  private double coneOuttakeSpd, cubeOuttakeSpd;
  public static DigitalInput clawSensor;
  public DoubleSolenoid clawExtenders;
  private double groundIntakeConePos = 101;
  private double groundIntakeCubePos = 93;
  private double loadZoneIntakePos = 0;
  private double highPos = 178;
  private double midPos = 130;
  private double lowPos = 99;//switch to 60 later
  private double startConfigPos = 29;
  private double driveConfigPos = 53;
  public static boolean isFinished = false;
  private double lastArmPos;
  public SparkMaxPIDController clawIntakePID;
  public RelativeEncoder clawIntakeEnc;
  public double intakeEncPos = 0;
  public int armStates;
  public DigitalInput cylinderSensor = new DigitalInput(1);
  /** Creates a new Claw. */
  public Claw() {
    coneOuttakeSpd = -0.75;
    cubeOuttakeSpd = 0.5;
    clawSensor = new DigitalInput(0);
    clawExtenders = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15);
    leftClawArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightClawArmMotor = new CANSparkMax(12, MotorType.kBrushless);
    clawIntake = new CANSparkMax(13, MotorType.kBrushless);

    leftClawArmMotor.restoreFactoryDefaults();
    rightClawArmMotor.restoreFactoryDefaults();
    clawIntake.restoreFactoryDefaults();

    rotateArmPID = leftClawArmMotor.getPIDController();
    clawIntakePID = clawIntake.getPIDController();
    throughBoreAbs = leftClawArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    clawIntakeEnc = clawIntake.getEncoder();

    rotateArmPID.setFeedbackDevice(throughBoreAbs);
    rotateArmPID.setP(0.006);
    rotateArmPID.setI(0);
    rotateArmPID.setD(0);

    clawIntakePID.setFeedbackDevice(clawIntakeEnc);
    clawIntakePID.setP(0.008);
    clawIntakePID.setI(0);
    clawIntakePID.setD(0);

    leftClawArmMotor.setIdleMode(IdleMode.kBrake);
    rightClawArmMotor.setIdleMode(IdleMode.kBrake);

    leftClawArmMotor.setSmartCurrentLimit(40);
    rightClawArmMotor.setSmartCurrentLimit(40);
    clawIntake.setSmartCurrentLimit(40);

    throughBoreAbs.setPositionConversionFactor(360);
    leftClawArmMotor.setInverted(true);
    rightClawArmMotor.setInverted(false);
    throughBoreAbs.setInverted(true);

    leftClawArmMotor.burnFlash();
    rightClawArmMotor.burnFlash();
    clawIntake.burnFlash();
    clawExtenders.set(Value.kReverse);

    //gamePieceDetect = new DigitalInput(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Absolute Position " + throughBoreAbs.getPosition());
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
  public void intakeStates(int clawStates, double speed, int counter){
    switch (clawStates){

      case 0: 
        clawIntake.set(0);
        clawExtenders.set(Value.kOff);
        break;

      case 1:
      //Rotate the arm to the ground position
        rotateArmPID.setReference(groundIntakeConePos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //If the arm is at the groud position and the claw sensor doesn't see anything extend pneumatics and run cone intake
        if(throughBoreAbs.getPosition() >= 84 /*&& clawSensor.get() == false*/){
          clawExtenders.set(Value.kForward);
          clawIntake.set(speed);
          //intakeEncPos = clawIntakeEnc.getPosition();
        }
        //If the sensor does see a game piece, for this case it is a cone, so set booleans to true except cube boolean
        //  else if(clawSensor.get() == true){
        //    clawExtenders.set(Value.kReverse);
        //    gamePiece = true;
        //    isCone = true;
        //    isCube = false;
        //    if (cylinderSensor.get() == true){
        //     rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
        //     rightClawArmMotor.follow(leftClawArmMotor, true);
        //    }
        //    if(counter > 500){
        //      isFinished = true;
        //    }
        //}
        break;

      case 2:
      //Rotate the arm to the ground position
        rotateArmPID.setReference(groundIntakeCubePos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //If the arm in the ground position and the sensor reads false then extend the pneumatics and run the cube intake
        if(throughBoreAbs.getPosition() >= 76 /*&& clawSensor.get() == false*/){
          clawExtenders.set(Value.kForward);
          clawIntake.set(speed);
          //intakeEncPos = clawIntakeEnc.getPosition();
        }
        //If the sensor does see a game piece, for this case it is a cube, so set booleans to true except cone boolean
        //  else if(clawSensor.get() == true){
        //    clawExtenders.set(Value.kReverse);
        //    gamePiece = true;
        //    isCone = false;
        //    isCube = true;
        //    if(cylinderSensor.get() == true){
        //     rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
        //     rightClawArmMotor.follow(leftClawArmMotor, true);
        //    }
        //    if(counter > 500){
        //      isFinished = true;
        //    }
        //  }
        break;
      case 3:
      //Rotate the arm to the loading zone position
        rotateArmPID.setReference(loadZoneIntakePos, ControlType.kPosition);
        //If the arm is in the loading zone position and the sensor reads false, then do not extend pneumatics and run intake for cone
        if(throughBoreAbs.getPosition() == loadZoneIntakePos && clawSensor.get() == false){
          clawExtenders.set(Value.kReverse);
          clawIntake.set(speed);
          intakeEncPos = clawIntakeEnc.getPosition();
        }
        //If the sensor does see a game piece, for this case it is a cone, so set booleans to true except cube boolean
        else if(clawSensor.get() == true){
          gamePiece = true;
          isCone = true;
          isCube = false;
          if(cylinderSensor.get() == true){
            rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
            rightClawArmMotor.follow(leftClawArmMotor, true);
          }
          if(counter > 500){
            isFinished = true;
          }
        }
        break;

      case 4:
      //Rotate the arm to the loading zone position
        rotateArmPID.setReference(loadZoneIntakePos, ControlType.kPosition);
        //If the arm is in the loading zone position and the sensor reads false, then do not extend pneumatics and run intake for cube
        if(throughBoreAbs.getPosition() == loadZoneIntakePos && clawSensor.get() == false){
          clawExtenders.set(Value.kReverse);
          clawIntake.set(speed);
        }
        //If the sensor does see a game piece, for this case it is a cube, so set booleans to true except cone boolean.
        else if(clawSensor.get() == true){
          gamePiece = true;
          isCone = false;
          isCube = true;
          if(cylinderSensor.get() == true){
            rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
            rightClawArmMotor.follow(leftClawArmMotor, true);
          }
          if(counter > 500){
            isFinished = true;
          }
        }
        break;
    }
  }
  public void setClawStates(int clawStates){

    clawExtenders.set(Value.kReverse);
    switch(clawStates){
      //Starting config state
      case 0:
        rotateArmPID.setReference(startConfigPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        clawIntake.set(0);
        clawExtenders.set(Value.kOff);
        isFinished = false;
        gamePiece = false;
        isCone = false;
        isCube = false;
        break;
      //Drive config state
      case 1:
        rotateArmPID.setReference(driveConfigPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        clawIntake.set(-0.05);
        break;
      //Set high arm position used for auto claw
      case 2:
        rotateArmPID.setReference(highPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;
      //Set mid arm position used for auto claw
      case 3:
        rotateArmPID.setReference(midPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;
      //Set low arm position used for auto claw
      case 4:
        rotateArmPID.setReference(lowPos, ControlType.kPosition);
        rightClawArmMotor.follow(leftClawArmMotor, true);
        if(throughBoreAbs.getPosition() >= 85){
          clawExtenders.set(Value.kForward);
        }
        //clawIntakePID.setReference(intakeEncPos, ControlType.kPosition);
        break;
    }
  }
  //To set the outtake speed with one button 
  public void setOuttakeSpd(){
    if(isCone == true && isCube == false){
      clawIntake.set(coneOuttakeSpd);
      if(clawSensor.get() == false){
        clawIntake.set(0);
        isCone = false;
        isFinished = true;
      }
    }
    else if(isCone == false && isCube == true){
      clawIntake.set(cubeOuttakeSpd);
      if(clawSensor.get() == false){
        clawIntake.set(0);
        isCube = false;
      }
    }
  }
  //Needed for auto claw
  public boolean getGamePiece(){
    return gamePiece;
  }
  //Needed for auto vision
  public static boolean getIsCone(){
    return isCone;
  }
  //Needed for auto vision
  public static boolean getIsCube(){
    return isCube;
  }
  
  //Override functions
  public void forward(){
    clawExtenders.set(Value.kForward);
  }
  public void reverse(){
    clawExtenders.set(Value.kReverse);
  }
  public void off(){
    clawExtenders.set(Value.kOff);
  }
  public void moveArm(double speed){
    // if(!cylinderSensor.get()){
    //   leftClawArmMotor.set(speed);
    //   rightClawArmMotor.set(speed);
    //   lastArmPos = throughBoreAbs.getPosition();
    // }
    System.out.println("Limit Switch is Pressed: " + cylinderSensor.get());
    // else{
    //   leftClawArmMotor.set(0);
    //   rightClawArmMotor.set(0);
    // }
  }
  public void intakeTester(double speed){
    clawIntake.set(speed);
  }
  public void intakeHold(){
    clawIntakePID.setReference(clawIntakeEnc.getPosition(), ControlType.kPosition);
  }
  public void holdArm(){
    rotateArmPID.setReference(lastArmPos, ControlType.kPosition);
    rightClawArmMotor.follow(leftClawArmMotor, true);
  }
  public void setArmState(int armState){
    armStates = armState;
  }
}
