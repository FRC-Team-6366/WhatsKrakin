package frc.robot.subsystems.extend;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.extend.ExtendIO.ExtendIOInputs;


public class Extend {
    private ExtendIO io;
    private ExtendIOInputs inputs = new ExtendIOInputsAutoLogged();

    public Extend(ExtendIO io) {
      this.io = io;
    }

    public void moveExtend(double rotations){
        io.moveExtend(rotations);
    }

    public boolean extendAtSetPoint(double atPosition){
      return io.extendAtSetPoint(atPosition);
    }


    /** updates extend values periodically */
    public void periodic() {
      io.updateInputs(inputs);
    //   Logger.processInputs("Extend", inputs);
    }

}