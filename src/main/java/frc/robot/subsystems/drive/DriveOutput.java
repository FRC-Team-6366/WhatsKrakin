package frc.robot.subsystems.drive;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.entech.subsystems.SubsystemOutput;

public class DriveOutput extends SubsystemOutput {
  private SwerveModulePosition[] modulePositions;
  private SwerveModuleState[] moduleStates;
  private ChassisSpeeds speeds;
  private Pose2d estimatedPose;

  @Override
  public void toLog() {
    Logger.recordOutput("DriveOutput/modulePositions", modulePositions);
    Logger.recordOutput("DriveOutput/moduleStates", moduleStates);
    Logger.recordOutput("DriveOutput/chassisSpeed", speeds);
    Logger.recordOutput("DriveOutput/estimatedPose", estimatedPose);
  }

  public SwerveModulePosition[] getModulePositions() {
    return this.modulePositions;
  }

  public void setModulePositions(SwerveModulePosition[] modulePositions) {
    this.modulePositions = modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    return this.moduleStates;
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    this.moduleStates = moduleStates;
  }

  public ChassisSpeeds getSpeeds() {
    return this.speeds;
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public Pose2d getEstimatedPose() {
    return this.estimatedPose;
  }

  public void setEstimatedPose(Pose2d estimatedPose) {
    this.estimatedPose = estimatedPose;
  }
}
