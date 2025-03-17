package frc.robot.processors.filters;

import frc.robot.subsystems.drive.DriveInput;

public class SquaringFilter implements DriveFilterI {
  @Override
  public DriveInput process(DriveInput input) {
    DriveInput processedInput = new DriveInput(input);

    processedInput
        .setXSpeed(Math.copySign(input.getXSpeed() * input.getXSpeed(), input.getXSpeed()));
    processedInput
        .setYSpeed(Math.copySign(input.getYSpeed() * input.getYSpeed(), input.getYSpeed()));
    processedInput.setRotation(Math.copySign(
        input.getRotation() * input.getRotation() * input.getRotation(), input.getRotation()));

    return processedInput;
  }
}
