package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;

public interface LedsIO {
    @AutoLog
        public static class LedsIOInputs {
            public boolean connected = false;
            // public Color ledColor = Color.kBlack;
    }


    public default void setColor() {
    }


    // public default void AllianceColor(){
    // }

    public default void updateInputs(LedsIOInputs inputs) {
    }
}
