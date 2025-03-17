package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.entech.subsystems.SubsystemInput;

public class DriveInput implements SubsystemInput {
  private double xSpeed;
  private double ySpeed;
  private double rot;
  private Rotation2d gyroAngle;
  private Pose2d latestOdometryPose;
  private String key = "driveInput";

  public DriveInput() {}

  public DriveInput(DriveInput template) {
    xSpeed = template.getXSpeed();
    ySpeed = template.getYSpeed();
    rot = template.getRotation();
    gyroAngle = template.getGyroAngle();
    latestOdometryPose = template.getLatestOdometryPose();
  }

  @Override
  public void fromLog(LogTable table) {
    xSpeed = table.get(key + "/xSpeed", 0.0);
    ySpeed = table.get(key + "/ySpeed", 0.0);
    rot = table.get(key + "/rot", 0.0);
    gyroAngle = table.get(key + "/gyroAngle", Rotation2d.fromDegrees(0));
    latestOdometryPose = table.get(key + "/pose", new Pose2d());
  }

  @Override
  public void toLog(LogTable table) {
    table.put(key + "/xSpeed", xSpeed);
    table.put(key + "/ySpeed", ySpeed);
    table.put(key + "/rot", rot);
    table.put(key + "/gyroAngle", gyroAngle);
    table.put(key + "/pose", latestOdometryPose);
  }

  public double getXSpeed() {
    return this.xSpeed;
  }

  public void setXSpeed(double xSpeed) {
    this.xSpeed = xSpeed;
  }

  public double getYSpeed() {
    return this.ySpeed;
  }

  public void setYSpeed(double ySpeed) {
    this.ySpeed = ySpeed;
  }

  public double getRotation() {
    return this.rot;
  }

  public void setRotation(double rot) {
    this.rot = rot;
  }

  public Rotation2d getGyroAngle() {
    return this.gyroAngle;
  }

  public void setGyroAngle(Rotation2d gyroAngle) {
    this.gyroAngle = gyroAngle;
  }

  public Pose2d getLatestOdometryPose() {
    return this.latestOdometryPose;
  }

  public void setLatestOdometryPose(Pose2d latestOdometryPose) {
    this.latestOdometryPose = latestOdometryPose;
  }

  public String getKey() {
    return this.key;
  }

  public void setKey(String key) {
    this.key = key;
  }
}