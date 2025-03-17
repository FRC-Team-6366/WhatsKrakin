package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends EntechSubsystem<DriveInput, DriveOutput> {
    private static final boolean ENABLED = true;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;

    public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    @Override
    public void initialize() {
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(DriveInput input) {
        Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(input.getXSpeed(), input.getYSpeed());

          // Apply rotation deadband
          double omega = input.getRotation();

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  linearVelocity.getY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  omega * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / DRIVE_BASE_RADIUS);
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        setDesiredModuleStates(setpointStates);
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public DriveOutput toOutputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public void setDesiredModuleStates(SwerveModuleState[] states) {
    }
}
