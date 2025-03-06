package frc.robot.subsystems.climb;

public class ClimbConstants {

  public static final int climbTalonId = 24;
  public static final boolean climbInvert = true;
  public static final boolean climbNeutralModeBrake = true;
  public static final double climbPeakVoltage = 2;
  public static final double climbGearRatio = 180;

  // Soft Limits
  public static final double climbForwardSoftLimit = 170; // in motor rotations
  public static final boolean climbForwardSoftLimitEnabled = false;

  public static final double climbReverseSoftLimit = 0;
  public static final boolean climbReverseSoftLimitEnabled = false;
}
