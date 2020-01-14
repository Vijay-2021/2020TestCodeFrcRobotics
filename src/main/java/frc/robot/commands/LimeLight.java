/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LookupTable;
import frc.robot.subsystems.DriveTrain;

public class LimeLight extends CommandBase {
  /**
   * Creates a new LimeLight.
   */
  NetworkTable limeTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry thor;
  DriveTrain drive_train_sys;
  double x_angle = 0.0;
 
  double[] inputs = {54,66,77,89,94,106,116};
  double[] outputs = {198,161.1,139,117,108.5,97,84};
  LookupTable thorDistanceTable;

  public LimeLight(DriveTrain trains) {
    drive_train_sys = trains;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(trains);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive_train_sys.resetNavX();
    try {
      thorDistanceTable = new LookupTable(inputs, outputs);
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    limeTable = NetworkTableInstance.getDefault().getTable("limelight");
  }
    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = limeTable.getEntry("tx");
    thor = limeTable.getEntry("thor");
    
    x_angle = tx.getDouble(0.0);
    double length = thor.getDouble(0.0);

    double navAngle = drive_train_sys.returnAngle();
    SmartDashboard.putNumber("nav angle", navAngle);
    SmartDashboard.putNumber("tx + nav angle", x_angle + navAngle);
    SmartDashboard.putNumber("tx - nav angle", x_angle - navAngle);
    SmartDashboard.putNumber("thor dist", length );
    SmartDashboard.putNumber("predicted distance",thorDistanceTable.get(length));
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
