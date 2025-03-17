package frc.robot.subsystems.drive;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.entech.subsystems.SubsystemOutput;

public class DriveOutput extends SubsystemOutput {
  private SwerveModulePosition[] modulePositions;
  private double[] rawAbsoluteEncoders;
  private double[] virtualAbsoluteEncoders;
  private SwerveModuleState[] moduleStates;
  private ChassisSpeeds speeds;

  @Override
  public void toLog() {
    Logger.recordOutput("DriveOutput/modulePositions", modulePositions);
    Logger.recordOutput("DriveOutput/rawAbsoluteEncoders", rawAbsoluteEncoders);
    Logger.recordOutput("DriveOutput/virtualAbsoluteEncoders", virtualAbsoluteEncoders);
    Logger.recordOutput("DriveOutput/moduleStates", moduleStates);
    Logger.recordOutput("DriveOutput/chassisSpeed", speeds);
  }

  public SwerveModulePosition[] getModulePositions() {
    return this.modulePositions;
  }

  public void setModulePositions(SwerveModulePosition[] modulePositions) {
    this.modulePositions = modulePositions;
  }

  public double[] getRawAbsoluteEncoders() {
    return this.rawAbsoluteEncoders;
  }

  public void setRawAbsoluteEncoders(double[] rawAbsoluteEncoders) {
    this.rawAbsoluteEncoders = rawAbsoluteEncoders;
  }

  public double[] getVirtualAbsoluteEncoders() {
    return this.virtualAbsoluteEncoders;
  }

  public void setVirtualAbsoluteEncoders(double[] virtualAbsoluteEncoders) {
    this.virtualAbsoluteEncoders = virtualAbsoluteEncoders;
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
}
