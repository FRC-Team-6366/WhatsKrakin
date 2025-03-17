package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.SensorsIO.SensorsIOInputs;
import org.littletonrobotics.junction.Logger;


public class Sensors extends SubsystemBase{
    private SensorsIO io;
    private SensorsIOInputs inputs = new SensorsIOInputsAutoLogged();

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
    //   Logger.processInputs("Sensors/CanRange", inputs);
    }

}
