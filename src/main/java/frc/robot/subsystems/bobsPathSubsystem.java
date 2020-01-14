/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team319.trajectory.Path;

import java.io.IOException;
import frc.paths.*;
import frc.robot.NavX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.sim.mockdata.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.Spark;
// import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class bobsPathSubsystem extends SubsystemBase {
  /**
   * Creates a new bobsPathSubsystem.
   */
  
  private static final int kTicksPerRev = 1;
  private static final double kWheelDiameter = 6.0 / 12.0 / 11.11 / Math.PI;
  private static final double kMaxVelocity = 7.5;

  private static final String pathName = "output/hi5";
  private CANSparkMax leftDriveMaster, rightDriveMaster, leftDriveFollower1, leftDriveFollower2, rightDriveFollower1,
  rightDriveFollower2;
  private double leftspeed, rightspeed;

  CANEncoder leftEncoder, rightEncoder;
  CANPIDController m_pidController,m_pidController2; 
  private boolean atDistances;
  private boolean atAngle;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2, maxRPM2, maxVel2, minVel2, maxAcc2, allowedErr2;


  private NavX navx;

  private Notifier notify;
  
  GenPathSetup pathinst = new GenPathSetup();
  private int currentLine =0;
  private double distancePerPulse = 60*(1/0.141); 
  straight10ft straight = new straight10ft();
  public bobsPathSubsystem() {
    
  }

  public void robotPeriodic() {
  }
  long currTime;
  public void autonomousInit() {
   

    
    navx.reset();
       
    currTime = System.currentTimeMillis();
  }

  public void autonomousPeriodic() {
    
  }

  
  public void teleopPeriodic() {
    leftDriveMaster.setIdleMode(IdleMode.kCoast);
    rightDriveMaster.setIdleMode(IdleMode.kCoast);

    leftDriveFollower1.setIdleMode(IdleMode.kCoast);
    leftDriveFollower2.setIdleMode(IdleMode.kCoast);
    rightDriveFollower1.setIdleMode(IdleMode.kCoast);
    rightDriveFollower2.setIdleMode(IdleMode.kCoast);
    try{notify.stop();}catch(Exception e){}
    leftDriveMaster.set(0);
    rightDriveMaster.set(0);
  }

  
  public void testPeriodic() {
  }

  private void followPath(){
    if(currentLine>straight.getPath().length-1){
      DriverStation.reportError("not working",false);
      leftDriveMaster.set(0);
      rightDriveMaster.set(0);
      notify.stop();
    }else{
     try{
       if(currentLine<2){
         navx.reset();
       }
      double m_currentHeading = navx.getAngle();
      double path_heading = straight.getPath()[currentLine][pathinst.heading()];
      double heading_difference = m_currentHeading+Math.toDegrees(path_heading);
      SmartDashboard.putNumber("heading difference",heading_difference);
      SmartDashboard.putNumber("your angle" , m_currentHeading);
      SmartDashboard.putNumber("path heading", Math.toDegrees(path_heading));
      SmartDashboard.putNumber("smaple heading diff",navx.sample_angle()+Math.toDegrees(path_heading));
      double rightVelocity = straight.getPath()[currentLine][pathinst.rightVel()];
      SmartDashboard.putNumber("Right velocity", rightVelocity);
      m_pidController.setReference(-straight.getPath()[currentLine][pathinst.leftVel()]*(120*11.111/Math.PI)+6.25*heading_difference,ControlType.kVelocity);
     // m_pidController.setReference(Rpath.get(currentLine).velocity*distancePerPulse ,ControlType.kVelocity);
      m_pidController2.setReference(straight.getPath()[currentLine][pathinst.rightVel()]*(120*11.11/Math.PI)+6.25*heading_difference,ControlType.kVelocity);
       
      
      currentLine++; 
     }catch(Exception e){
      
     }
    }
    
  }
  public void setBrake(){
    leftDriveMaster.setIdleMode(IdleMode.kBrake);
    rightDriveMaster.setIdleMode(IdleMode.kBrake);
    leftDriveFollower1.setIdleMode(IdleMode.kBrake);
    leftDriveFollower2.setIdleMode(IdleMode.kBrake); 
    rightDriveFollower1.setIdleMode(IdleMode.kBrake);
    rightDriveFollower2.setIdleMode(IdleMode.kBrake);

  }
  public void disabledInit(){
   
    DriverStation.reportError("change time " + Long.toString(System.currentTimeMillis()-currTime),false);
    setBrake();
  }
  public double capSpeed(double speed){
    double cap = 0.5;
    if((speed)>cap){
      speed = cap;
    }
    if((-speed)<cap){
      speed = -cap;
    }
    return speed;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
