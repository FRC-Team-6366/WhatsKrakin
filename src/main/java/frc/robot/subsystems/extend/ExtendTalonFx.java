// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extend;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ExtendTalonFx implements ExtendIO {

  private final TalonFX _extendMotorK;
  private final CANcoder _extendCANCoder;

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> torqueCurrentAmps;
  private StatusSignal<Temperature> tempCelsius;
  private StatusSignal<Angle> absolutePositionRotations;
  private StatusSignal<AngularVelocity> absoluteVelocityRotationsPerSecond;

  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public ExtendTalonFx() {
    _extendMotorK = new TalonFX(ExtendConstants.extendTalonId);
    _extendCANCoder = new CANcoder(ExtendConstants.extendCANCoderId);

    positionRotations = _extendMotorK.getPosition();
    velocityRotationsPerSecond = _extendMotorK.getVelocity();
    voltage = _extendMotorK.getMotorVoltage();
    supplyCurrentAmps = _extendMotorK.getSupplyCurrent();
    torqueCurrentAmps = _extendMotorK.getTorqueCurrent();
    tempCelsius = _extendMotorK.getDeviceTemp();
    absolutePositionRotations = _extendCANCoder.getAbsolutePosition();
    absoluteVelocityRotationsPerSecond = _extendCANCoder.getVelocity();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(
            ExtendConstants.extendInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive
        )
        .withNeutralMode(ExtendConstants.extendNeutralModeBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.Slot0.kP = ExtendConstants.kP;
    cfg.Slot0.kI = ExtendConstants.kI;
    cfg.Slot0.kD = ExtendConstants.kD;
    cfg.Slot0.kG = ExtendConstants.kG;
    cfg.Slot0.kS = ExtendConstants.kS;
    cfg.Slot0.kV = ExtendConstants.kV;
    cfg.Slot0.kA = ExtendConstants.kA;
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = ExtendConstants.extendForwardSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ExtendConstants.extendForwardSoftLimit;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = ExtendConstants.extendReverseSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ExtendConstants.extendReverseSoftLimit;
    
    cfg.Feedback.SensorToMechanismRatio = 1;
    cfg.Feedback.RotorToSensorRatio = ExtendConstants.extendGearRatio;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = _extendCANCoder.getDeviceID(); //Add this when adding an EnCoder gives it its ID
    
    // Motion Magic
    cfg.MotionMagic.MotionMagicAcceleration = ExtendConstants.motionMagicAcceleration;
    cfg.MotionMagic.MotionMagicCruiseVelocity = ExtendConstants.MotionMagicCruiseVelocity;

    // voltage limits
    // cfg.Voltage.PeakForwardVoltage = ExtendConstants.extendPeakVoltage;
    // cfg.Voltage.PeakReverseVoltage = -ExtendConstants.extendPeakVoltage;

    _extendMotorK.setPosition(_extendCANCoder.getAbsolutePosition().getValueAsDouble());
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionRotations,
        velocityRotationsPerSecond,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        absolutePositionRotations,
        absoluteVelocityRotationsPerSecond);

    _extendMotorK.optimizeBusUtilization(0.0, 1.0);
    _extendCANCoder.optimizeBusUtilization(0.0, 1.0);

    _extendMotorK.getConfigurator().apply(cfg);

    positionVoltageRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    _extendMotorK.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void runVolts(double volts) {
    _extendMotorK.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void extendToLength(double extendLengthInches, double currentPivotRotations) {
    double targetExtendRotations =
        (extendLengthInches
                - (currentPivotRotations * ExtendConstants.spoolCircumferenceInches)
                - ExtendConstants.extendOffsetInchAtZeroDegrees)
            / (ExtendConstants.spoolCircumferenceInches);
    _extendMotorK.setControl(
        positionVoltageRequest.withPosition(targetExtendRotations).withSlot(0));
  }

  @Override
  public double getLength(double currentPivotRotations) {
    return ExtendConstants.extendOffsetInchAtZeroDegrees
        + (_extendMotorK.getPosition().getValueAsDouble() * currentPivotRotations)
            * (ExtendConstants.spoolCircumferenceInches);
  }

  @Override
  public void updateConfig() {}

  @Override
  public void updateInputs(ExtendIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRotationsPerSecond,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                absolutePositionRotations,
                absoluteVelocityRotationsPerSecond)
            .isOK();

    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.positionInches = positionRotations.getValueAsDouble() * (2 * Math.PI * 2);
    inputs.velocityRotationsPerSecond = velocityRotationsPerSecond.getValueAsDouble();
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
}
