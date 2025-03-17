package frc.robot.subsystems.vacuum;

import org.littletonrobotics.junction.AutoLog;

public interface VacuumIO {
    @AutoLog
        public static class VacuumIOInputs {
            public boolean connected = false;
            public double servoPositionAngle = 0.0;
        }
  
    public default void setServoPosition(double angle) {
    }

    public default void runVolts(double volts) {
    }

    public default void updateInputs(VacuumIOInputs inputs) {
    }
    
    
}
