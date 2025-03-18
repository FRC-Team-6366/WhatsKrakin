package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase{
       private LedsIO io;
    // private LedsIO inputs = new LedsIOInputsAutoLogged(); 

    public void periodic() {
        // io.updateInputs(inputs);
        // Logger.processInputs("Extend", inputs);
      }

    public Leds(LedsIO io) {
      this.io = io;
    }

      public void setColor() {
        io.setColor();
    }
}
