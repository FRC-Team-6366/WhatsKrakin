package frc.robot.subsystems.pigeon;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.entech.subsystems.SubsystemOutput;

public class PigeonOutput extends SubsystemOutput {
    private boolean connected = false;
    private Rotation2d yawPosition = new Rotation2d();
    private double yawVelocityRadPerSec = 0.0;
    private double[] odometryYawTimestamps = new double[] {};
    private Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    @Override
    public void toLog() {
        Logger.recordOutput("PigeonOutput/connected", connected);
        Logger.recordOutput("PigeonOutput/yawPosition", yawPosition);
        Logger.recordOutput("PigeonOutput/yawVelocityRadPerSec", yawVelocityRadPerSec);
        Logger.recordOutput("PigeonOutput/odometryYawTimestamps", odometryYawTimestamps);
        Logger.recordOutput("PigeonOutput/odometryYawPositions", odometryYawPositions);
    }

    public boolean isConnected() {
        return this.connected;
    }

    public void setConnected(boolean connected) {
        this.connected = connected;
    }

    public Rotation2d getYawPosition() {
        return this.yawPosition;
    }

    public void setYawPosition(Rotation2d yawPosition) {
        this.yawPosition = yawPosition;
    }

    public double getYawVelocityRadPerSec() {
        return this.yawVelocityRadPerSec;
    }

    public void setYawVelocityRadPerSec(double yawVelocityRadPerSec) {
        this.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }

    public double[] getOdometryYawTimestamps() {
        return this.odometryYawTimestamps;
    }

    public void setOdometryYawTimestamps(double[] odometryYawTimestamps) {
        this.odometryYawTimestamps = odometryYawTimestamps;
    }

    public Rotation2d[] getOdometryYawPositions() {
        return this.odometryYawPositions;
    }

    public void setOdometryYawPositions(Rotation2d[] odometryYawPositions) {
        this.odometryYawPositions = odometryYawPositions;
    }
}
