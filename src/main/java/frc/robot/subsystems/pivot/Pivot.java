package frc.robot.subsystems.pivot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public class Pivot extends SubsystemBase {
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
      // Logger.processInputs("Arm/Pivot", inputs);
    }
  

}
