package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotIOTalonFX implements PivotIO {
    private final TalonFX _pivotkMotor;
    private final CANcoder _pivotCANcoder;
    
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> absoluteVelocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

    private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0).withSlot(0);

    public PivotIOTalonFX(){
        _pivotkMotor = new TalonFX(PivotConstants.pivotID, "roborio");
        _pivotCANcoder = new CANcoder(PivotConstants.pivotEncoderID, "roborio");

        position = _pivotkMotor.getPosition();
        velocity = _pivotkMotor.getVelocity();
        voltage = _pivotkMotor.getMotorVoltage();
        supplyCurrentAmps = _pivotkMotor.getSupplyCurrent();
        torqueCurrentAmps = _pivotkMotor.getTorqueCurrent();
        tempCelsius = _pivotkMotor.getDeviceTemp();
        absolutePosition = _pivotCANcoder.getAbsolutePosition();
        absoluteVelocity = _pivotCANcoder.getVelocity();


        TalonFXConfiguration cfg = new TalonFXConfiguration();
          cfg.MotorOutput
        .withInverted(PivotConstants.pivotMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(PivotConstants.pivotBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(PivotConstants.pivotCurrentLimitEnabled)
        .withSupplyCurrentLimit(PivotConstants.pivotSupplyCurrentLimits);
        cfg.ClosedLoopGeneral.ContinuousWrap = PivotConstants.pivotContinousWrap; 
          Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = PivotConstants.pivotkS; // Add 0.25 V output to overcome static friction
    slot0.kV = PivotConstants.pivotkV; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = PivotConstants.pivotkA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = PivotConstants.pivotkP; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = PivotConstants.pivotkI; // No output for integrated error
    slot0.kD = PivotConstants.pivotkD; // A velocity error of 1 rps results in 0.5 V output
           FeedbackConfigs fdb = cfg.Feedback;
           cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = PivotConstants.pivotSoftLimitHighEnabled;
           cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.pivotSoftLimitHigh;
           cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = PivotConstants.pivotSoftLimitLowEnabled;
           cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.pivotSoftLimitLow;
           cfg.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(PivotConstants.pivotMMCruiseVelocity)) // 5 (mechanism) rotations per second cruise
           .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(PivotConstants.pivotMMAcceleration)) // Take approximately 0.5 seconds to reach max vel
           // Take approximately 0.1 seconds to reach max accel 
           .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(PivotConstants.pivotMMJerk));
           fdb.SensorToMechanismRatio = PivotConstants.pivotSensorRatio;
           fdb.RotorToSensorRatio = PivotConstants.pivotRotorGearRatio;
             fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; //rezero CANcoder
             fdb.FeedbackRemoteSensorID = _pivotCANcoder.getDeviceID();

             BaseStatusSignal.setUpdateFrequencyForAll(
              50,
              position,
              velocity,
              voltage,
              supplyCurrentAmps,
              torqueCurrentAmps,
              tempCelsius,
              absolutePosition,
              absoluteVelocity);
        _pivotkMotor.optimizeBusUtilization(0.0, 1.0);
        _pivotCANcoder.optimizeBusUtilization(0.0, 1.0);
           _pivotkMotor.setPosition(_pivotCANcoder.getAbsolutePosition().getValueAsDouble());
           _pivotkMotor.getConfigurator().apply(cfg);}
           
@Override
    public void movePivot(double rotations){
      _pivotkMotor.setControl(mmVolts.withPosition(rotations).withSlot(0));
    }

    // public void runVolts(double volts){
    //     _pivotkMotor.setControl(mmVolts(volts).withSlot(0));
    //   }

    @Override
    public boolean pivotAtSetPoint(double atPosition){
    boolean positionTrueFalse;
    double difference = Math.abs(atPosition -  _pivotCANcoder.getAbsolutePosition().getValueAsDouble()); //gets difference of the two
    positionTrueFalse = difference < 0.1; //sets the difference and how much it should be
    SmartDashboard.putBoolean("ArmAtSetPoint", positionTrueFalse); //prints whether its true or false
    return positionTrueFalse; //returns true or false
    }

    public void toggleBrake(boolean brakeMode) {
        _pivotkMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
      }

      @Override
      public void updateInputs(PivotIOInputs inputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                    position,
                    velocity,
                    voltage,
                    supplyCurrentAmps,
                    torqueCurrentAmps,
                    tempCelsius,
                    absolutePosition,
                    absoluteVelocity)
                .isOK();
    
        inputs.positionAngle = Units.rotationsToDegrees(position.getValueAsDouble());
        inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());
    
        inputs.appliedVoltage = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
        inputs.temperatureCelsius =tempCelsius.getValueAsDouble(); // dont add absolute position or coder stuff
      }    
}
