// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extend;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ExtendIO {

  @AutoLog
  public static class ExtendIOInputs {

    public boolean connected = false;
    public double positionInches = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double positionRotations = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void setBrakeMode(boolean brakeMode) {}

  /**
   * @param inch extends to distance in inches from start
   */
  public default void extendToLength(double extendLengthInches, double currentPivotRotations) {}

  public default void runVolts(double volts) {}

  public default double getLength(double currentPivotRotations) {
    return 0;
  }

  public default void updateConfig() {}

  public default void updateInputs(ExtendIOInputs inputs) {}
}
