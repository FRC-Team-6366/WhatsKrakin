package frc.robot.subsystems.pivot;

public final class PivotConstants {
    public static final int pivotID = 21;
    public static final int pivotEncoderID = 23;

    public static boolean pivotBrakeMode = false;
    public static boolean pivotMotorInverted = false;
    public static boolean pivotContinousWrap = false;
    public static boolean pivotSoftLimitHighEnabled = false;
    public static boolean pivotSoftLimitLowEnabled = false;

    public static double pivotSupplyCurrentLimits = 40;
    public static boolean pivotCurrentLimitEnabled = true;

    public static double pivotkP = 65; //
    public static double pivotkI = 0; // 0
    public static double pivotkD = 0.5; // 0.5
    public static double pivotkS = 0.25; //0
    public static double pivotkV = 0.12; // 0.12
    public static double pivotkA = 0.01; //0.01
    public static double pivotkG = 0; // 0
    public static double pivotSoftLimitLow = 0; // Not used
    public static double pivotSoftLimitHigh = 0; // Not used
    public static double pivotSensorRatio = 1;
    public static double pivotRotorGearRatio = 60;

    public static double pivotMMCruiseVelocity = 0.5;
    public static double pivotMMAcceleration = 1;
    public static double pivotMMJerk = 100;
}
