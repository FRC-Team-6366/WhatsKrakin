// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ExtensionTalonFx implements ExtensionIO {

  private final TalonFX _extendMotorK;
  private final CANcoder _extendCANcoder;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> absoluteVelocity;

  private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  public ExtensionTalonFx() {
    _extendMotorK = new TalonFX(SuperStructureConstants.ExtensionId);
    _extendCANcoder = new CANcoder(SuperStructureConstants.ExtensionEncoderID);

    position = _extendMotorK.getPosition();
    velocity = _extendMotorK.getVelocity();
    voltage = _extendMotorK.getMotorVoltage();
    supplyCurrentAmps = _extendMotorK.getSupplyCurrent();
    torqueCurrentAmps = _extendMotorK.getTorqueCurrent();
    tempCelsius = _extendMotorK.getDeviceTemp();
    absolutePosition = _extendCANcoder.getAbsolutePosition();
    absoluteVelocity = _extendCANcoder.getVelocity();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(SuperStructureConstants.ExtensionInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond
      .of(SuperStructureConstants.extendCruiseVelocity * SuperStructureConstants.extendRotationsToInches))
    .withMotionMagicAcceleration(RotationsPerSecondPerSecond
      .of(SuperStructureConstants.extendCruiseAcceleration * SuperStructureConstants.extendRotationsToInches));
    cfg.Slot0.kP = SuperStructureConstants.ExtensionP;
    cfg.Slot0.kI = SuperStructureConstants.ExtensionI;
    cfg.Slot0.kD = SuperStructureConstants.ExtensionD;
    cfg.Slot0.kG = SuperStructureConstants.ExtensionG;
    cfg.Slot0.kV = SuperStructureConstants.ExtensionV;
    cfg.Slot0.kS = SuperStructureConstants.ExtensionS;
    cfg.Slot0.kA = SuperStructureConstants.ExtensionA;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperStructureConstants.extensionSoftLimitHigh;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperStructureConstants.extensionSoftLimitLow;
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    cfg.Feedback.SensorToMechanismRatio = 1; 
    cfg.Feedback.RotorToSensorRatio = SuperStructureConstants.extensionGearRatio;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = _extendCANcoder.getDeviceID(); //Add this when adding an EnCoder gives it its ID
    
    // voltage limits
    cfg.Voltage.PeakForwardVoltage = SuperStructureConstants.extensionPeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -SuperStructureConstants.extensionPeakVoltage;
    // spotless:on

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
    _extendMotorK.optimizeBusUtilization(0.0, 1.0);

    _extendCANcoder.optimizeBusUtilization(0.0, 1.0);
    _extendMotorK.getConfigurator().apply(cfg);
    _extendMotorK.setPosition(_extendCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void extendToDistance(double inch) {
    double target = inch / SuperStructureConstants.extendRotationsToInches;
    _extendMotorK.setControl(mmVolts.withPosition(target));
  }

  @Override
  public void runVolts(double volts) {
    _extendMotorK.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public double getExtend() {
    return _extendMotorK.getPosition().getValueAsDouble()
        * SuperStructureConstants.extendRotationsToInches;
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
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

    inputs.positionInch =
        position.getValueAsDouble() * SuperStructureConstants.extendRotationsToInches;
    inputs.positionExtensionRotation = position.getValueAsDouble();
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();
  }

  // TODO add absolute encoder
  // TODO add input loggging
}
