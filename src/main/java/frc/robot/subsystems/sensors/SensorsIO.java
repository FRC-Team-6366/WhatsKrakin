package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;


public interface SensorsIO {

    @AutoLog
        public static class SensorsIOInputs {
            public boolean connected = false;
            public double sensorValue = 0.0;
            public double sensorOneDistance = 0.0;
            public double sensorTwoDistance = 0.0;
        
}
    public default void updateInputs(SensorsIOInputs inputs) {}


    public default boolean sensorDetectCoral(){
        return false;
    }
    

    public default double askSensorDistance(double distance){
        return 0;
    }

}
