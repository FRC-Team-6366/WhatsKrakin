package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedsIOLED implements LedsIO{
    private Spark _led;
    // private AddressableLEDBuffer _buffer;
    // private Color currentColor;
    // private Color currentInput;
    // private Color setColor;
    // private Color color;



     public LedsIOLED() {
        _led = new Spark(1);
    // Reuse buffer

    // Default to a length of 60, start empty output

    // Length is expensive to set, so only set it once, then just update data

    // _buffer = new AddressableLEDBuffer(60);

        // _buffer = new AddressableLEDBuffer(239);
        // _led.setLength(_buffer.getLength());
        // _led.start();
     }


        @Override
        public void setColor() {
            _led.set(0.57);
            // red.applyTo(_buffer);
            // _led.setData(_buffer);
            // for (var i = 0; i < _buffer.getLength(); i++) {
            //   _buffer.setLED(i, c);
            // }
            // _led.setData(_buffer);
          }

    //     @Override 
    //     public void AllianceColor(Color ){
    //        Optional<Alliance> allianceColors = DriverStation.getAlliance();
    //         if(allianceColors.isPresent() && allianceColors.get() == Alliance.Red) {
    //                led.set(0.61);
    //         } else {
    //               led.set(0.81);
            
    //         }
    // }


    @Override 
        public void updateInputs(LedsIOInputs inputs) {
        }
}
