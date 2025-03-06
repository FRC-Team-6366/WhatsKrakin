package frc.robot.util;

public class SetpointConstants {
  public record Setpoint(double pivotAngleDegrees, double extendLengthInches) {}

  public static Setpoint HOME = new Setpoint(0, 16);
  public static Setpoint CORAL = new Setpoint(0, 16);
  public static Setpoint L1 = new Setpoint(0, 16);
  public static Setpoint L2 = new Setpoint(0, 16);
  public static Setpoint L3 = new Setpoint(0, 16);
  public static Setpoint L4 = new Setpoint(0, 16);
  public static Setpoint HANG = new Setpoint(0, 16);
}
