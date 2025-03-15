package frc.robot.subsystems.pivot;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public class Pivot {
    private PivotIO io;
    private PivotIOInputs inputs = new PivotIOInputsAutoLogged();
  
    public Pivot(PivotIO io) {
      this.io = io;
    }
  
    public void movePivot(double rotations){
        io.movePivot(rotations);
    }
  
    
    public boolean pivotAtSetPoint(double atPosition){
      return io.pivotAtSetPoint(atPosition);
      }
  

    /** updates arm values periodically */
    public void periodic() {
      io.updateInputs(inputs);
    //   Logger.processInputs("Pivot", inputs);
    }
  

}
