package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import frc.entech.commands.EntechCommand;
import frc.robot.util.DriveInputSupplier;
import frc.robot.processors.DriveInputProcessor;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveCommand extends EntechCommand {
  private final DriveSubsystem drive;
  private final DriveInputProcessor inputProcessor;
  private final DriveInputSupplier driveInputSource;

  public DriveCommand(DriveSubsystem drive, DriveInputSupplier driveInputSource) {
    super(drive);
    this.drive = drive;
    this.driveInputSource = driveInputSource;
    this.inputProcessor = new DriveInputProcessor();
  }

  @Override
  public void end(boolean interrupted) {
    DriveInput stop = new DriveInput(driveInputSource.getDriveInput());

    stop.setRotation(0.0);
    stop.setXSpeed(0.0);
    stop.setYSpeed(0.0);

    drive.updateInputs(stop);
  }

  @Override
  public void execute() {
    DriveInput input = inputProcessor.processInput(driveInputSource.getDriveInput());

    drive.updateInputs(input);
  }

  @Override
  public void initialize() {
    DriveInput stop = new DriveInput(driveInputSource.getDriveInput());

    stop.setRotation(0.0);
    stop.setXSpeed(0.0);
    stop.setYSpeed(0.0);

    drive.updateInputs(stop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
