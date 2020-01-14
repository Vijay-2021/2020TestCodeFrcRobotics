/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
*Pid tuning methods tried
* random guess: didn't work
* double p until no oscilatoin, then double i

*/
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NavX;

public class DriveTrain extends SubsystemBase {
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
  public DriveTrain() {
    //leftDriveMaster.setControlFramePeriodMs(2); test this later when have time 
    DriverStation.reportError("this is vijay code",false);
    leftDriveMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftDriveFollower1 = new CANSparkMax(2, MotorType.kBrushless);
    leftDriveFollower2 = new CANSparkMax(3, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(4, MotorType.kBrushless);
    rightDriveFollower1 = new CANSparkMax(5, MotorType.kBrushless);
    rightDriveFollower2 = new CANSparkMax(6, MotorType.kBrushless);
    leftDriveMaster.restoreFactoryDefaults();
    rightDriveMaster.restoreFactoryDefaults();
    leftDriveFollower1.restoreFactoryDefaults();
    leftDriveFollower2.restoreFactoryDefaults();
    rightDriveFollower1.restoreFactoryDefaults();
    rightDriveFollower2.restoreFactoryDefaults();
    
    leftDriveFollower1.follow(leftDriveMaster);
    leftDriveFollower2.follow(leftDriveMaster);
    rightDriveFollower1.follow(rightDriveMaster);
    rightDriveFollower2.follow(rightDriveMaster);

    leftEncoder = leftDriveMaster.getEncoder();
    rightEncoder = rightDriveMaster.getEncoder();
  
    leftDriveMaster.setIdleMode(IdleMode.kCoast);
    rightDriveMaster.setIdleMode(IdleMode.kCoast);
    
    leftDriveFollower1.setIdleMode(IdleMode.kCoast);
    leftDriveFollower2.setIdleMode(IdleMode.kCoast);
    //rightDriveFollower1.setIdleMode(IdleMode.kCoast);
    rightDriveFollower2.setIdleMode(IdleMode.kCoast);
    navx = new NavX();
    m_pidController = leftDriveMaster.getPIDController();
    m_pidController2 = rightDriveMaster.getPIDController();


    // PID coefficients
    double p = 0.000001;//0.001
    double i = 0.000;
    double d  = 0.00;
    double f = 0.00018;
  
    kP = p; 
    kI = d;
    kD = i; 
  
    kIz = 0; 
    kFF = 0.000178; //loeft = .000178 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5500;

    // Smart Motion Coefficients
    maxVel = 5500; // rpm
    maxAcc = 5000;

    kP2 = p;//0.000065
    kI2 = i;// 0.0000029
    kD2 = d; //0.00095
    kIz2 = 0; 
    kFF2 = 0.0001823;// working ff for 6ft/s/s0.000182; //right motor
    kMaxOutput2 = 1; 
    kMinOutput2 = -1;
    maxRPM2 = 5500;

    // Smart Motion Coefficients
    maxVel2 = 5500; // rpm
    maxAcc2 = 5000;
    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    //leftDriveMaster.setControlFramePeriodMs(periodMs);
    //m_pidController.
    m_pidController2.setP(kP2);
    m_pidController2.setI(kI2);
    m_pidController2.setD(kD2);
    m_pidController2.setIZone(kIz2);
    m_pidController2.setFF(kFF2);
    m_pidController2.setOutputRange(kMinOutput2, kMaxOutput2);
    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_pidController2.setSmartMotionMaxVelocity(maxVel2, smartMotionSlot);
    m_pidController2.setSmartMotionMinOutputVelocity(minVel2, smartMotionSlot);
    m_pidController2.setSmartMotionMaxAccel(maxAcc2, smartMotionSlot);
    m_pidController2.setSmartMotionAllowedClosedLoopError(allowedErr2, smartMotionSlot);

    

    // gyro/pigeon/navx

  }
  public double returnAngle(){
    return navx.getAngle();
  }
  
  public void setVelocity(double left_velocity,double right_velocity, double heading){
    m_pidController.setReference(-left_velocity+heading,ControlType.kVelocity);
    m_pidController2.setReference(right_velocity+heading,ControlType.kVelocity);
  }
  public double returnVelocity(){
    return leftEncoder.getVelocity();
  }
  public void setBrake(){
    leftDriveMaster.setIdleMode(IdleMode.kBrake);
    rightDriveMaster.setIdleMode(IdleMode.kBrake);
    leftDriveFollower1.setIdleMode(IdleMode.kBrake);
    leftDriveFollower2.setIdleMode(IdleMode.kBrake);
    rightDriveFollower1.setIdleMode(IdleMode.kBrake);
    rightDriveFollower2.setIdleMode(IdleMode.kBrake);

  }
  public void setCoast(){
    leftDriveMaster.setIdleMode(IdleMode.kCoast);
    rightDriveMaster.setIdleMode(IdleMode.kCoast);
    leftDriveFollower1.setIdleMode(IdleMode.kCoast);
    leftDriveFollower2.setIdleMode(IdleMode.kCoast);
    rightDriveFollower1.setIdleMode(IdleMode.kCoast);
    rightDriveFollower2.setIdleMode(IdleMode.kCoast);
  }
  public void arcadeDrive(double speed ,double turn){
    leftDriveMaster.set(speed + turn);
    rightDriveMaster.set(-speed+turn);
  }

  @Override
  public void periodic() {
    
  }
  public void resetNavX() {
    navx.reset();
  }

}
