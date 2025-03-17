package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
    public static class PivotIOInputs{
      public boolean connected = false;

      public double positionAngle = 0.0;
      public double velocityRPM = 0.0;
  

      public double appliedVoltage = 0.0;
      public double supplyCurrentAmps = 0.0;
      public double torqueCurrentAmps = 0.0;
      public double temperatureCelsius = 0.0;
    }
  public default void updateInputs(PivotIOInputs inputs) {
  } // Updates inputs


  public default void movePivot(double rotations){
  }

  
  public default boolean pivotAtSetPoint(double atPosition){
    return false;
  }

  public default void toggleBrake(boolean brakeMode) {
  } // Toggles brake mode
}
