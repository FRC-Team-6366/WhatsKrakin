package frc.robot.processors.filters;

import frc.robot.subsystems.drive.DriveInput;

public interface DriveFilterI {
  public DriveInput process(DriveInput input);
}
