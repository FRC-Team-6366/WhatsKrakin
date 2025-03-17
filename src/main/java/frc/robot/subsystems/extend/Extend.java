package frc.robot.subsystems.extend;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.extend.ExtendIO.ExtendIOInputs;


public class Extend extends SubsystemBase{
    private ExtendIO io;
    private ExtendIOInputs inputs = new ExtendIOInputsAutoLogged();

      /** updates extend values periodically */
      public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Extend", inputs);
      }

    public Extend(ExtendIO io) {
      this.io = io;
    }

    public void moveExtend(double rotations){
        io.moveExtend(rotations);
    }

    public boolean extendAtSetPoint(double atPosition){
      return io.extendAtSetPoint(atPosition);
    }


}