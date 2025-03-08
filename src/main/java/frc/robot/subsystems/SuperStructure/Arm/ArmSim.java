// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ArmSim implements ArmIO {
  private PIDController controller;
  private SingleJointedArmSim arm;
  private ArmFeedforward feedforward;

  public ArmSim() {
    arm =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            6.6,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(15), 5),
            Units.inchesToMeters(15),
            Units.degreesToRadians(-90),
            Units.degreesToRadians(270),
            true,
            0);
    controller = new PIDController(0.1, 0, 0.01);
    feedforward = new ArmFeedforward(0.001, 2.395, 0);
  }

  @Override
  public void setAngle(double angle) {
    double goTo;
    if ((angle * 360 < SuperStructureConstants.PrepAngle
            && Units.radiansToDegrees(arm.getAngleRads()) > SuperStructureConstants.PrepAngle + 2)
        || (angle * 360 > SuperStructureConstants.PrepAngle
            && Units.radiansToDegrees(arm.getAngleRads())
                < SuperStructureConstants.PrepAngle
                    - 2)) { // protect against rotating the short way
      goTo = SuperStructureConstants.PrepAngle;
    } else {
      goTo = angle * 360;
    }
    double pid = controller.calculate(getAngle().getDegrees(), goTo);
    double feed =
        feedforward.calculate(
            getAngle().getRadians(), Units.degreesToRadians(goTo) - getAngle().getRadians());
    double volts =
        MathUtil.clamp(
            pid + feed,
            -SuperStructureConstants.anglePeakVoltage,
            SuperStructureConstants.anglePeakVoltage);
    arm.setInputVoltage(volts);
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(arm.getAngleRads());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    arm.update(0.02);
    inputs.connected = true;
    inputs.positionAngle = getAngle().getDegrees();
    inputs.velocityDegPerSec =
        Units.radiansPerSecondToRotationsPerMinute(arm.getVelocityRadPerSec());

    inputs.appliedVoltage = arm.getInput(0);
    inputs.supplyCurrentAmps = arm.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = arm.getCurrentDrawAmps();
    inputs.temperatureCelsius = 2120; // the atmosphere might catch fire
  }
}
