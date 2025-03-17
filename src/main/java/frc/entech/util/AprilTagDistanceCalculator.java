package frc.entech.util;

public final class AprilTagDistanceCalculator {

    private AprilTagDistanceCalculator() {
    }

    public static double calculateCurrentDistanceInches(AprilTagDistanceCalibration calibration, double newTagWidthPixels) {
        if (calibration == null) {
            throw new IllegalArgumentException("Calibration cannot be null.");
        }
        if (calibration.getTagWidthPixels() <= 0) {
            throw new IllegalArgumentException("Calibration tag width must be greater than zero.");
        }
        if (calibration.getDistanceFeet() <= 0) {
            throw new IllegalArgumentException("Calibration distance must be greater than zero.");
        }
        if (newTagWidthPixels <= 0) {
            throw new IllegalArgumentException("New tag width in pixels must be greater than zero.");
        }

        // Perform calculation
        return (calibration.getTagWidthPixels() * calibration.getDistanceFeet()) / newTagWidthPixels;
    }
}
