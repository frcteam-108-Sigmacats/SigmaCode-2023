// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveSetUp;

public class SwerveSubsystem extends SubsystemBase {
  //Construction of each module

  //Front Left
  private SwerveModule frontLeft = new SwerveModule(SwerveSetUp.fLDriveMotor, SwerveSetUp.isFLDriveMotorReversed, 
  SwerveSetUp.frontLDriveKP, SwerveSetUp.frontLDriveKI, SwerveSetUp.frontLDriveKD, SwerveSetUp.fLTurnMotor, SwerveSetUp.isFLTurnMotorReversed, 
  SwerveSetUp.frontLTurnKP, SwerveSetUp.frontLTurnKI, SwerveSetUp.frontLTurnKD, SwerveSetUp.fLChassisAngleOffset);
  //Front Right
  private SwerveModule frontRight = new SwerveModule(SwerveSetUp.fRDriveMotor, SwerveSetUp.isFRDriveMotorReversed, 
  SwerveSetUp.frontRDriveKP, SwerveSetUp.frontRDriveKI, SwerveSetUp.frontRDriveKD, SwerveSetUp.fRTurnMotor, SwerveSetUp.isFRTurnMotorReversed, 
  SwerveSetUp.frontRTurnKP, SwerveSetUp.frontRTurnKI, SwerveSetUp.frontRTurnKD, SwerveSetUp.fRChassisAngleOffset);
  //Back Left
  private SwerveModule backLeft = new SwerveModule(SwerveSetUp.bLDriveMotor, SwerveSetUp.isBLDriveMotorReversed, 
  SwerveSetUp.backLDriveKP, SwerveSetUp.backLDriveKI, SwerveSetUp.backLDriveKD, SwerveSetUp.bLTurnMotor, SwerveSetUp.isBLTurnMotorReversed, 
  SwerveSetUp.backLTurnKP, SwerveSetUp.backLTurnKI, SwerveSetUp.backLTurnKD, SwerveSetUp.bLChassisAngleOffset);
  //Back Right
  private SwerveModule backRight = new SwerveModule(SwerveSetUp.bRDriveMotor, SwerveSetUp.isBRDriveMotorReversed, 
  SwerveSetUp.backRDriveKP, SwerveSetUp.backRDriveKI, SwerveSetUp.backRDriveKD, SwerveSetUp.bRTurnMotor, SwerveSetUp.isBRTurnMotorReversed, 
  SwerveSetUp.backRTurnKP, SwerveSetUp.backRTurnKI, SwerveSetUp.backRTurnKD, SwerveSetUp.bRChassisAngleOffset);

  private static SwerveDriveOdometry odometry;
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(1);
  //Getting community zone points to make auto claw
  private Pose2d bAllCommFarPos = new Pose2d(4.67, 0, null);
  private Pose2d bAllCommMinPos = new Pose2d(0,5.25, null);
  private Pose2d bAllCommMidPos = new Pose2d(4.69, 3.77, null);
  private Pose2d rAllCommMinPos = new Pose2d(16.5, 5, null);
  private Pose2d rAllCommMidPos = new Pose2d(11.94, 3.77, null);
  private Pose2d rAllCommFarPos = new Pose2d(11.94, 0, null);
  //The positions we will assign depending on which alliance color we are on 
  private Pose2d commMinPos = new Pose2d();
  private Pose2d commMidPos = new Pose2d();
  private Pose2d commFarPos = new Pose2d();

  private SlewRateLimiter xLim = new SlewRateLimiter(5);
  private SlewRateLimiter yLim = new SlewRateLimiter(5);
  private SlewRateLimiter rotLim = new SlewRateLimiter(0.5);
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    zeroHeading();
    gyro.configFactoryDefault();
    odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(), getModulesPosition());
  }
  //Gets the robots heading based on gyro
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  //Resets the gyros heading
  public void zeroHeading(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //If we are on blue alliance we will assign the blue alliance community zone positions to the community zone positions
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      commMinPos = bAllCommMinPos;
      commMidPos = bAllCommMidPos;
      commFarPos = bAllCommFarPos;
    }
    //Else if we are on the red alliance we will assign the red alliance community zone positions to the community zone positions
    else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      commMinPos = rAllCommMinPos;
      commMidPos = rAllCommMidPos;
      commFarPos = rAllCommFarPos;
    }
    odometry.update(
      getHeading(), getModulesPosition());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Robot Degree: ", gyro.getAngle());
    SmartDashboard.putNumber("Robot Pose X: ", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y: ", getPose().getY());
    SmartDashboard.putNumber("FrontL Angle: ", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FrontR Angle: ", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackL Angle: ", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackR Angle: ", backRight.getState().angle.getDegrees());
  }
  //Sets the swerve module states based on the parameters values mainly used for joystick drive
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                xLim.calculate(translation.getX()), 
                                yLim.calculate(translation.getY()), 
                                rotLim.calculate(rotation), 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                xLim.calculate(translation.getX()), 
                                yLim.calculate(translation.getY()), 
                                rotLim.calculate(rotation))
                            );
    setModuleStates(swerveModuleStates);
  }
  //Sets the modules states and assures that the speed of the modules never goes beyond the maxDriveSpeed
  public void setModuleStates(SwerveModuleState[] desiredState){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.maxDriveSpeed);
    frontLeft.setDesiredState(desiredState[0]);
    frontRight.setDesiredState(desiredState[1]);
    backLeft.setDesiredState(desiredState[2]);
    backRight.setDesiredState(desiredState[3]);
  }
  //Gets the modules positions 
  public SwerveModulePosition[] getModulesPosition(){
    SwerveModulePosition[] position = new SwerveModulePosition[4];
    position[0] = frontLeft.getPosition();
    position[1] = frontRight.getPosition();
    position[2] = backLeft.getPosition();
    position[3] = backRight.getPosition();
    return position;
  }
  //Resets odometry ot the specified pose
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getHeading(), getModulesPosition(), pose);
  }

  public void resetAutoOdometry(Pose2d pose){
    odometry.resetPosition(getYaw(), getModulesPosition(), pose);
  }
  public void resetEncoders(){
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
  }
  //Used for field centric drive. If gyro is inversed it will go the opposite way of the way it was positioned
  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (SwerveSetUp.invertedGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
  }
  //Gets the pitch of the gyro needed for auto balance
  public double getPitch(){
    return gyro.getPitch();
  }
  //To tell us if we can extend our arm automaticall to the high peg positions
  public boolean canArmExtend(){
    //First need to see if we have a game piece
    if(Claw.gamePiece == true){
      //Next to see which alliance we are on to switch operating signs
      if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        //If our robots position are above the y position above the charge station we will get the smaller length of the community zone
        if(getPose().getY() > commMidPos.getY()){
          //If the robot position is withing the community zone will return true
          if(getPose().getX() <= commMinPos.getX() && getPose().getY() <= commMinPos.getY()){
            return true;
          }
          else{
            return false;
          }
        }
        //If our robots position is lower than the y position below the charge station we will get the bigger length of the community zone
        else if(getPose().getY() < commMidPos.getY()){
          //If the robots position is inside the community zone we will return true
          if(getPose().getX() <= commFarPos.getX()){
            return true;
          }
          else{
            return false;
          }
        }
      }
      //Else if we are in the red alliance we will get different operations to see if we are within the community zone
      else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
        if(getPose().getY() > commMidPos.getY()){
          if(getPose().getX() >= commMinPos.getX() && getPose().getY() <= commMinPos.getY()){
            return true;
          }
          else{
            return false;
          }
        }
        else if(getPose().getY() < commMidPos.getY()){
          if(getPose().getX() >= commFarPos.getX()){
            return true;
          }
          else{
            return false;
          }
        }
      }
    }
    return false;
  }
}
