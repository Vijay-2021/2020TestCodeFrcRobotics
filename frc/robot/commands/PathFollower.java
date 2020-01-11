/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.paths.GenPathSetup;
import frc.paths.straight10ft;
import frc.robot.subsystems.DriveTrain;

public class PathFollower extends CommandBase {
  GenPathSetup pathinst; 
  straight10ft straight10;
  int currentLine = 0;
  private final DriveTrain trains; 
  /**
   * Creates a new PathFollower.
   */
  public PathFollower(DriveTrain train) {
    // Use addRequirements() here to declare subsystem dependencies.
    trains = train;
    addRequirements(train);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathinst = new GenPathSetup();
    straight10 = new straight10ft();
    trains.resetNavX();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_currentHeading = trains.returnAngle();
    double path_heading = straight10.getPath()[currentLine][pathinst.heading()];
    double heading_difference = m_currentHeading+Math.toDegrees(path_heading);
    SmartDashboard.putNumber("heading difference",heading_difference);
    SmartDashboard.putNumber("your angle" , m_currentHeading);
    SmartDashboard.putNumber("path heading", Math.toDegrees(path_heading));
    SmartDashboard.putNumber("smaple heading diff",Math.toDegrees(heading_difference));
    double rightVelocity = straight10.getPath()[currentLine][pathinst.rightVel()];
    SmartDashboard.putNumber("Right velocity", rightVelocity);
    trains.setVelocity(straight10.getPath()[currentLine][pathinst.leftVel()]*(120*11.111/Math.PI),6.25*heading_difference);
    
    currentLine++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentLine>=straight10.getPath().length-1);
  }
}
