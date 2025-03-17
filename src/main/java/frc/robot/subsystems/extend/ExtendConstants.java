package frc.robot.subsystems.extend;

public final class ExtendConstants {
    public static final int extendID = 20;
    public static final int extendEncoderID = 22;
    public static boolean extendBrakeMode = false;
    public static boolean extendMotorInverted = false;
    public static boolean extendContinousWrap = false;
    public static boolean extendSoftLimitHighEnabled = false;
    public static boolean extendSoftLimitLowEnabled = false;

    public static double extendSupplyCurrentLimits = 40;
    public static boolean extendCurrentLimitEnabled = true;

    public static double extendkP = 15; //15
    public static double extendkI = 0; //0
    public static double extendkD = 0; //0
    public static double extendkS = 0; //0
    public static double extendkV = 0; //0
    public static double extendkA = 0; //0
    public static double extendkG = 0; //0
    public static double extendSoftLimitLow = 0; // Not used
    public static double extendSoftLimitHigh = 0; // Not used
    public static double extendSensorRatio = 1;
    public static double extendRotorGearRatio = 18;

    public static double extendMMCruiseVelocity = 11;
    public static double extendMMAcceleration = 11;
    public static double extendMMJerk = 100;
}