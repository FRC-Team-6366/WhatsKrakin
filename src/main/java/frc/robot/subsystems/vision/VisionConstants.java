// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "photonvisionFrontRight";
  public static String camera1Name = "photonvisionFrontLeft";
  public static String camera2Name = "photonvisionRearRight";
  public static String camera3Name = "photonvisionRearLeft";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(7.23),
          Units.inchesToMeters(-6.184),
          Units.inchesToMeters(14.734),
          new Rotation3d(
              Units.degreesToRadians(1), Units.degreesToRadians(11), Units.degreesToRadians(6)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(7.23),
          Units.inchesToMeters(6.184),
          Units.inchesToMeters(14.734),
          new Rotation3d(
              Units.degreesToRadians(-1), Units.degreesToRadians(11), Units.degreesToRadians(-6)));
  public static Transform3d robotToCamera2 = // update, changed from hang interference
      new Transform3d(
          Units.inchesToMeters(2.55),
          Units.inchesToMeters(-7.109),
          Units.inchesToMeters(31.0),
          new Rotation3d(
              Units.degreesToRadians(12.6),
              Units.degreesToRadians(-18),
              Units.degreesToRadians(-135)));
  public static Transform3d robotToCamera3 =
      new Transform3d(
          Units.inchesToMeters(2.55),
          Units.inchesToMeters(7.109),
          Units.inchesToMeters(31.0),
          new Rotation3d(
              Units.degreesToRadians(-12.6),
              Units.degreesToRadians(-18),
              Units.degreesToRadians(135)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
