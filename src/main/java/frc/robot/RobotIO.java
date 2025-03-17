package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.DriveInputSupplier;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveOutput;
import frc.robot.subsystems.pigeon.PigeonOutput;

public class RobotIO implements DriveInputSupplier {
    private static final RobotIO instance = new RobotIO();

    public static RobotIO getInstance() {
        return instance;
    }

    public static void processInput(LoggableInputs in) {
        Logger.processInputs(in.getClass().getSimpleName(), in);
    }

    private RobotIO() {}

    @Override
    public DriveInput getDriveInput() {
        DriveInput di = new DriveInput();
        di.setGyroAngle(RobotIO.getInstance().getLatesPigeonOutput().getYawPosition());
        // di.setLatestOdometryPose(latestOdometryPose);
        di.setKey("initialRaw");
        di.setRotation(0.0);
        di.setXSpeed(0.0);
        di.setYSpeed(0.0);
        processInput(di);
        return di;
    }

    public void updateDriveOutput(DriveOutput driveOutput) {
        this.driveOutput = driveOutput;
        this.driveOutput.log();
    }

    public void updatePigeonOutput(PigeonOutput pigeonOutput) {
        this.pigeonOutput = pigeonOutput;
        this.pigeonOutput.log();
    }

    public DriveOutput getLatestDriveOutput() {
        return this.driveOutput;
    }

    public PigeonOutput getLatesPigeonOutput() {
        return this.pigeonOutput;
    }

    private PigeonOutput pigeonOutput = new PigeonOutput();
    private DriveOutput driveOutput = new DriveOutput();
}
