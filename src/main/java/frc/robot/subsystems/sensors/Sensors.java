package frc.robot.subsystems.sensors;

import frc.robot.subsystems.sensors.SensorsIO.SensorsIOInputs;


public class Sensors {
    private SensorsIO io;
    private SensorsIOInputs inputs = new SensorIOInputsAutoLogged();

    public Sensors(SensorsIO io) {
      this.io = io;
    }

    public boolean sensorDetectCoral(){
        return io.sensorDetectCoral();
    }
    

    public double askSensorDistance(double distance){
        return io.askSensorDistance(distance);
    }

    /** updates extend values periodically */
    public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Sensors/CanRange", inputs);
    }

}
