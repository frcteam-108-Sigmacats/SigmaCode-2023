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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private SparkMaxPIDController drivePID;
  private SparkMaxPIDController turningPID;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder turnEncoder;
  private double chassisAngleOffset;

  private SwerveModuleState desiredState = new SwerveModuleState();
  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorID, boolean driveMotorReversed, double kDP, double kDI, double kDD, 
  int turnMotorID, boolean turnMotorReversed, double kTP, double kTI, double kTD, double absolutePositionOffset) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    //Factory reset to configure the speed controllers. Useful for when swapping out speed controllers
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    //Set up of encoders and PID controllers
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivePID = driveMotor.getPIDController();
    turningPID = turnMotor.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    turningPID.setFeedbackDevice(turnEncoder);

    //Modules angle relative to the chassis
    chassisAngleOffset = absolutePositionOffset;

    drivePID.setP(kDP);
    drivePID.setI(kDI);
    drivePID.setD(kDD);

    turningPID.setP(kTP);
    turningPID.setI(kTI);
    turningPID.setD(kTD);

    //Setting motors to brake mode to prevent easy movements of the motors when the robot is on
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);

    //Setting current limit to make sure the Spark Max's do not burn out Unit:AMPS
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrent);
    turnMotor.setSmartCurrentLimit(SwerveConstants.turnCurrent);

    //Applying conversion factors for position and velocity
    driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRot2Meters);
    driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRPM2MPS);
    turnEncoder.setPositionConversionFactor(SwerveConstants.kTurnEncoderRot2Rad);
    turnEncoder.setVelocityConversionFactor(SwerveConstants.kTurnEncoderRPM2RadPerSec);

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    chassisAngleOffset = absolutePositionOffset;
    driveEncoder.setPosition(0);
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
  }

  //Getting each Modules states velocity and angular position with encoders
  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - chassisAngleOffset));
  }

  //Getting each modules position. Necessary for odometry appliance
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() - chassisAngleOffset));
  }

  //Setting modules desired state
  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngleOffset));

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turnEncoder.getPosition()));

    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    this.desiredState = desiredState;
    
  }

  //Resetting drive motors encoders
  public void resetEncoder(){
    driveEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}