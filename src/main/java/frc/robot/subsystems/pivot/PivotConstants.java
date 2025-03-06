package frc.robot.subsystems.pivot;

public class PivotConstants {
  public static final int pivotTalonId = 20;
  public static final int pivotCANCoderId = 23;
  public static final boolean pivotInvert = false;
  public static final boolean pivotNeutralModeBrake = false;
  public static final double pivotPeakVoltage = 6;
  public static final double pivotGearRatio = 36;

  // Soft Limits
  public static final boolean pivotForwardSoftLimitEnabled = false;
  public static final double pivotForwardSoftLimit = 0; // update
  public static final boolean pivotReverseSoftLimitEnabled = false;
  public static final double pivotReverseSoftLimit = 0; // update

  // PID
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;
  public static double kG = 0;
  public static double kS = 0;
  public static double kV = 0;
  public static double kA = 0;

  public static double kGExtendFactor = 1; //proportional increase to kG for each inch of extension, update
}
