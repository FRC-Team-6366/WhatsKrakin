package frc.robot.subsystems.climb;

public class ClimbConstants {

  public static final int climbTalonId = 24;
  public static final boolean climbInvert = true; // confirm
  public static final boolean climbNeutralModeBrake = true;
  public static final double climbPeakVoltage = 2;
  public static final double climbGearRatio = 180;

  // Command Stop Points
  public static final double climbPrepAngleDegrees = 0; // update
  public static final double climbHangAngleDegrees = 0; // update

  // Soft Limits
  public static final boolean climbForwardSoftLimitEnabled = false;
  public static final double climbForwardSoftLimit = 0; // update
  public static final boolean climbReverseSoftLimitEnabled = false;
  public static final double climbReverseSoftLimit = 0; // update
}
