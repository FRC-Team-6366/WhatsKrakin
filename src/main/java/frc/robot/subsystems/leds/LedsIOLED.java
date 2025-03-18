package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.sensors.SensorsIO.SensorsIOInputs;

public class LedsIOLED implements LedsIO{
//      private AddressableLED _led;

//     LedsIOLED(){
//        _led = new AddressableLED(9);


//     // Reuse buffer

//     // Default to a length of 60, start empty output

//     // Length is expensive to set, so only set it once, then just update data

// // Create the buffer
// AddressableLEDBuffer _buffer = new AddressableLEDBuffer(120);

// // // Create the view for the section of the strip on the left side of the robot.
// // // This section spans LEDs from index 0 through index 59, inclusive.
// // AddressableLEDBufferView _left = _buffer.createView(0, 59);

// // // The section of the strip on the right side of the robot.
// // // This section spans LEDs from index 60 through index 119, inclusive.
// // // This view is reversed to cancel out the serpentine arrangement of the
// // // physical LED strip on the robot.
// // AddressableLEDBufferView _right = _buffer.createView(60, 119).reversed();

// // Create an LED pattern that sets the entire strip to solid red
// // LEDPattern red = LEDPattern.solid(Color.kRed);

// LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kYellow)
// .scrollAtRelativeSpeed(Percent.per(Second).of(0.5));
// LEDPattern pattern = base.reversed();


// // Apply the LED pattern to the data buffer
// pattern.applyTo(_buffer);

// // Write the data to the LED strip
// _led.setData(_buffer);

//     _led.start();
//     }

//        @Override 
//         public boolean detectCoral(){
//             return false;
//         }

//     @Override
//         public double askSensorDistance(double distance){
//             return 0;
//         }

//     @Override 
//         public void updateInputs(LedsIOInputs inputs) {
 
//         }
}
