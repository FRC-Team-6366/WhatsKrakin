package frc.robot.subsystems.pigeon;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

public class PigeonSubsystem extends EntechSubsystem<PigeonInput, PigeonOutput> {
    private static final boolean ENABLED = true;

    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;
    private StatusSignal<AngularVelocity> yawVelocity;

    @Override
    public void initialize() {
        if (ENABLED) {
            pigeon = new Pigeon2(
                TunerConstants.DrivetrainConstants.Pigeon2Id,
                TunerConstants.DrivetrainConstants.CANBusName);
            yaw = pigeon.getYaw();
            yawVelocity = pigeon.getAngularVelocityZWorld();
            pigeon.getConfigurator().apply(new Pigeon2Configuration());
            pigeon.getConfigurator().setYaw(0.0);
            yaw.setUpdateFrequency(DriveSubsystem.ODOMETRY_FREQUENCY);
            yawVelocity.setUpdateFrequency(50.0);
            pigeon.optimizeBusUtilization();
            yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(PigeonInput input) {
        throw new UnsupportedOperationException("No Possible Input");
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public PigeonOutput toOutputs() {
        PigeonOutput output = new PigeonOutput();

        if (ENABLED) {
            output.setConnected(BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK));
            output.setYawPosition(Rotation2d.fromDegrees(yaw.getValueAsDouble()));
            output.setYawVelocityRadPerSec(Units.degreesToRadians(yawVelocity.getValueAsDouble()));
            output.setOdometryYawPositions(
                yawPositionQueue.stream()
                    .map((Double value) -> Rotation2d.fromDegrees(value))
                    .toArray(Rotation2d[]::new)
            );
            output.setOdometryYawTimestamps(yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray());

            yawTimestampQueue.clear();
            yawPositionQueue.clear();
        }

        return output;
    }
}
