package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.RobotIO;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;

public class DriveSubsystem extends EntechSubsystem<DriveInput, DriveOutput> {
    private static final boolean ENABLED = true;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;
    static final Lock odometryLock = new ReentrantLock();

    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

    private SwerveModulePosition[] lastModulePositions;
    private Rotation2d rawGyroRotation = new Rotation2d();

    public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    private ChassisSpeeds lastChassisSpeeds;

    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        new Pose2d()
    );

    @Override
    public void initialize() {
        frontLeft = new SwerveModule(TunerConstants.FrontLeft);
        frontRight = new SwerveModule(TunerConstants.FrontRight);
        rearLeft = new SwerveModule(TunerConstants.BackLeft);
        rearRight = new SwerveModule(TunerConstants.BackRight);

        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::driveWithChassisSpeeds,

            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            }
        );
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            }
        );
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(DriveInput input) {
        Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(input.getXSpeed(), input.getYSpeed());

        double omega = input.getRotation();

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                linearVelocity.getY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                omega * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / DRIVE_BASE_RADIUS);

        driveWithChassisSpeeds(speeds);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        lastChassisSpeeds = speeds;
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
        DriveOutput output = new DriveOutput();
        if (ENABLED) {
            output.setModulePositions(getModulePositions());
            output.setModuleStates(new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(),
                rearLeft.getState(), rearRight.getState()});
            output.setSpeeds(lastChassisSpeeds);
            output.setEstimatedPose(poseEstimator.getEstimatedPosition());
        }

        return output;
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
        if (ENABLED) {
            frontLeft.setDesiredState(states[0]);
            frontRight.setDesiredState(states[1]);
            rearLeft.setDesiredState(states[2]);
            rearRight.setDesiredState(states[3]);
        }
    }

    public void setX() {
        if (ENABLED) {
          frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        if (ENABLED) {
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            rearLeft.getPosition(), rearRight.getPosition()};
        } else {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,Rotation2d.fromDegrees(0.0)),
            new SwerveModulePosition(0.0,Rotation2d.fromDegrees(0.0)),
            new SwerveModulePosition(0.0,Rotation2d.fromDegrees(0.0)),
            new SwerveModulePosition(0.0,Rotation2d.fromDegrees(0.0)) };
        }
    }

    private SwerveModulePosition[][] getModuleOdometryPositions() {
        return new SwerveModulePosition[][] {
            frontLeft.getOdometryPositions(), frontRight.getOdometryPositions(), rearLeft.getOdometryPositions(), rearRight.getOdometryPositions()};
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        frontLeft.runPeriodic();
        frontRight.runPeriodic();
        rearLeft.runPeriodic();
        rearRight.runPeriodic();
        odometryLock.unlock();

        // Update odometry

        if (lastModulePositions == null) {
            lastModulePositions = getModulePositions();
        }
        double[] sampleTimestamps = frontLeft.getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        SwerveModulePosition[][] modulePositions = getModuleOdometryPositions();
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex][i].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex][i].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex][i];
            }

            
            // Update gyro angle
            if (RobotIO.getInstance().getLatestPigeonOutput() != null) {
                // Use the real gyro angle
                rawGyroRotation = RobotIO.getInstance().getLatestPigeonOutput().getYawPosition();
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, new SwerveModulePosition[] {
                modulePositions[0][i], modulePositions[1][i], modulePositions[2][i], modulePositions[3][i]
            });
        }
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotIO.getInstance().getLatestPigeonOutput().getYawPosition(), getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return lastChassisSpeeds;
    }
}
