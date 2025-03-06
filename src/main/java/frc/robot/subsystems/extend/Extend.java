// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extend;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Extend extends SubsystemBase {
  private ExtendIO io;
  private ExtendIOInputsAutoLogged inputs = new ExtendIOInputsAutoLogged();
  private double setPointLengthInches;

  public Extend(ExtendIO io) {
    this.io = io;
  }

  /** updates Extend values periodically */
  public void extendPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extend", inputs);
  }

  public void extendToLength(double extendLengthInch, double currentPivotRotations) {
    io.extendToLength(extendLengthInch, currentPivotRotations);
    this.setPointLengthInches = extendLengthInch;
  }

  public double getLength(double currentPivotRotations) {
    return io.getLength(currentPivotRotations);
  }

  public void holdLength(double currentPivotRotations) {
    io.extendToLength(setPointLengthInches, currentPivotRotations);
  }

  public boolean atSetPoint(double currentPivotRotations) {
    return Math.abs(io.getLength(currentPivotRotations) - setPointLengthInches)
        < 1; // might need to add check for high velocity
  }

  public void updateConfig() {
    io.updateConfig();
  }
}
