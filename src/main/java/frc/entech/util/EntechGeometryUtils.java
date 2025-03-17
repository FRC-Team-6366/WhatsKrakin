package frc.entech.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Tools for arimetic of wpilib geometry
 *
 * @author ahietkamp
 */
public final class EntechGeometryUtils {

  /**
   * Averages two Pose3d objests
   *
   * @param poseA
   * @param poseB
   * @return Average Pose
   */
  public static Pose3d averagePose3d(Pose3d poseA, Pose3d poseB) {
    Rotation3d rotation =
        new Rotation3d((poseA.getRotation().getX() + poseB.getRotation().getX()) / 2,
            (poseA.getRotation().getY() + poseB.getRotation().getY()) / 2,
            (poseA.getRotation().getZ() + poseB.getRotation().getZ()) / 2);

    return new Pose3d((poseA.getX() + poseB.getX()) / 2, (poseA.getY() + poseB.getY()) / 2,
        (poseA.getZ() + poseB.getZ()) / 2, rotation);
  }

  public static Pose2d averagePose2d(Pose2d poseA, Pose2d poseB) {
    Rotation2d rotation = Rotation2d
        .fromDegrees((poseA.getRotation().getDegrees() + poseB.getRotation().getDegrees()) / 2);

    return new Pose2d((poseA.getX() + poseB.getX()) / 2, (poseA.getY() + poseB.getY()) / 2,
        rotation);
  }

  private EntechGeometryUtils() {}
}
