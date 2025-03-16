package frc.robot.subsystems.extend;

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
import frc.robot.subsystems.pivot.PivotConstants;


public class ExtendIOTalonFX implements ExtendIO{
  private final TalonFX _extendKMotor;
  private final CANcoder _extendCANcoder;

  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> absoluteVelocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0).withSlot(0);

  public ExtendIOTalonFX() {
  _extendKMotor = new TalonFX(20, "roborio");
  _extendCANcoder = new CANcoder(23,"roborio");

  position = _extendKMotor.getPosition();
  velocity = _extendKMotor.getVelocity();
  voltage = _extendKMotor.getMotorVoltage();
  supplyCurrentAmps = _extendKMotor.getSupplyCurrent();
  torqueCurrentAmps = _extendKMotor.getTorqueCurrent();
  tempCelsius = _extendKMotor.getDeviceTemp();
  absolutePosition = _extendCANcoder.getAbsolutePosition();
  absoluteVelocity = _extendCANcoder.getVelocity();

  TalonFXConfiguration cfg = new TalonFXConfiguration();
     cfg.MotorOutput
        .withInverted(PivotConstants.pivotMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(ExtendConstants.extendBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(ExtendConstants.extendCurrentLimitEnabled)
        .withSupplyCurrentLimit(ExtendConstants.extendSupplyCurrentLimits);
        cfg.ClosedLoopGeneral.ContinuousWrap = ExtendConstants.extendContinousWrap; 
  Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = ExtendConstants.extendkS; // Add 0.25 V output to overcome static friction
    slot0.kV = ExtendConstants.extendkV; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = ExtendConstants.extendkA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = ExtendConstants.extendkP; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = ExtendConstants.extendkI; // No output for integrated error
    slot0.kD = ExtendConstants.extendkD; // A velocity error of 1 rps results in 0.5 V output
           FeedbackConfigs fdb = cfg.Feedback;
             cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = ExtendConstants.extendSoftLimitHighEnabled;
           cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ExtendConstants.extendSoftLimitHigh; //Add soft limits
           cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = ExtendConstants.extendSoftLimitLowEnabled;
           cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ExtendConstants.extendSoftLimitLow; //Add soft limits
           cfg.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(ExtendConstants.extendMMCruiseVelocity)) // 5 (mechanism) rotations per second cruise
           .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ExtendConstants.extendMMAcceleration)) // Take approximately 0.5 seconds to reach max vel
           // Take approximately 0.1 seconds to reach max accel 
           .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ExtendConstants.extendMMJerk));
           fdb.SensorToMechanismRatio = ExtendConstants.extendSensorRatio;
           fdb.RotorToSensorRatio = ExtendConstants.extendRotorGearRatio;
             fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; //rezero CANcoder
             fdb.FeedbackRemoteSensorID = _extendCANcoder.getDeviceID();
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
            _extendKMotor.setPosition(_extendCANcoder.getPosition().getValueAsDouble());
           _extendKMotor.getConfigurator().apply(cfg);
  }

  @Override
  public void moveExtend(double rotations){
    _extendKMotor.setControl(mmVolts.withPosition(rotations).withSlot(0));
  }

//   public void runVolts(double volts){
//     extend.setControl(voltageOut.withOutput(volts));
//   }

  public boolean extendAtSetPoint(double atPosition){
    boolean positionTrueFalse;
    double difference = Math.abs(atPosition -  _extendKMotor.getPosition().getValueAsDouble()); //gets difference of the two
    positionTrueFalse = difference < 0.1; //sets the difference and how much it should be
       SmartDashboard.putBoolean("ExtendAtSetPoint", positionTrueFalse); //prints whether its true or false
    return positionTrueFalse; //returns true or false
    }

        @Override
      public void updateInputs(ExtendIOInputs inputs) {
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
        inputs.temperatureCelsius = tempCelsius.getValueAsDouble(); // dont add absolute position or coder stuff
      }    

}
