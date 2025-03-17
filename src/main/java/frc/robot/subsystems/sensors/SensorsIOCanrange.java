package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;


public class SensorsIOCanrange implements SensorsIO{
    private final CANrange sensorOne;
    private final CANrange sensorTwo;

      private final StatusSignal<Distance> sensorOneDistance;
      private final StatusSignal<Distance> sensorTwoDistance;

    public SensorsIOCanrange(){
        sensorOne = new CANrange(0);
        sensorTwo = new CANrange(1);

        sensorOneDistance = sensorOne.getDistance();
        sensorTwoDistance = sensorTwo.getDistance();    

        CANrangeConfiguration config = new CANrangeConfiguration();
  
        BaseStatusSignal.setUpdateFrequencyForAll(
              50,
              sensorOneDistance,
        sensorTwoDistance);
        sensorOne.optimizeBusUtilization(0.0, 1.0);
        sensorTwo.optimizeBusUtilization(0.0, 1.0);
        sensorOne.getConfigurator().apply(config);
        sensorTwo.getConfigurator().apply(config);
    }
   


    @Override 
        public boolean sensorDetectCoral(){
            return false;
        }

    @Override
        public double askSensorDistance(double distance){
            return 0;
        }

    @Override 
        public void updateInputs(SensorsIOInputs inputs) {
            inputs.connected =
            BaseStatusSignal.refreshAll(
                    sensorOneDistance, sensorTwoDistance)
                .isOK();
            inputs.connected = sensorOne.isConnected() && sensorTwo.isConnected();
            inputs.sensorOneDistance = sensorOne.getDistance().getValueAsDouble();
            inputs.sensorTwoDistance = sensorTwo.getDistance().getValueAsDouble();
        }
}
