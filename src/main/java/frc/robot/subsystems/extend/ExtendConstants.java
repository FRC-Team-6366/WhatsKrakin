package frc.robot.subsystems.extend;

public class ExtendConstants {
  public static final int extendTalonId = 20;
  public static final int extendCANCoderId = 23;
  public static final boolean extendInvert = false;
  public static final boolean extendNeutralModeBrake = false;
  public static final double extendPeakVoltage = 6;
  public static final double extendGearRatio = 36; // confirm
  public static final double spoolCircumferenceInches = 2 * Math.PI * 2;
  public static final double extendOffsetInchAtZeroDegrees = 12; // update
  public static final double safePivotLengthInches = 12; // update

  // Soft Limits
  public static final boolean extendForwardSoftLimitEnabled = false;
  public static final double extendForwardSoftLimit = 0; // update
  public static final boolean extendReverseSoftLimitEnabled = false;
  public static final double extendReverseSoftLimit = 0; // update

  // PID
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;
  public static double kG = 0;
  public static double kS = 0;
  public static double kV = 0;
  public static double kA = 0;

  // Motion Magic
  public static double motionMagicAcceleration = 0;
  public static double MotionMagicCruiseVelocity = 0;
}
