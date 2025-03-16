package frc.robot.subsystems.extend;

import org.littletonrobotics.junction.AutoLog;

public interface ExtendIO {
    @AutoLog
public static class ExtendIOInputs {
    public boolean connected = false;

    public double positionAngle = 0.0;
    public double velocityRPM = 0.0;
  
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
}
public default void updateInputs(ExtendIOInputs inputs) {} // Updates inputs

public default void moveExtend(double rotations){}

public default boolean extendAtSetPoint(double atPosition){
    return false;
    }
}
