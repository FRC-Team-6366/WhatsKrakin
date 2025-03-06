// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extend;

import edu.wpi.first.math.filter.Debouncer;
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
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extend", inputs);
  }

  public void extendToLength(double extendLengthInches, double currentPivotRotations) {
    io.extendToLength(extendLengthInches, currentPivotRotations);
    this.setPointLengthInches = extendLengthInches;
  }

  public double getLength(double currentPivotRotations) {
    return io.getLength(currentPivotRotations);
  }

  public void holdLength(double currentPivotRotations) {
    io.extendToLength(setPointLengthInches, currentPivotRotations);
  }

  public boolean atSetPoint(double currentPivotRotations) {
    Debouncer setpointDebouncer = new Debouncer(0.5);
    return setpointDebouncer.calculate(
        Math.abs(io.getLength(currentPivotRotations) - setPointLengthInches) < 1);
  }

  public void updateConfig() {
    io.updateConfig();
  }
}
