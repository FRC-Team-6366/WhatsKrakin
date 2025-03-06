package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean connected = false;

    public double positionRotations = 0.0;
    public double angleDegrees = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void runVolts(double volts) {}

  public default double getAngle() {
    return 0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}
}
