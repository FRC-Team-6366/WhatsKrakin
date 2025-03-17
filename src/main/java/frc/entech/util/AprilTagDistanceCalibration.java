package frc.entech.util;

public class AprilTagDistanceCalibration {
    private double screenWidthPixels;
    private double screenHeightPixels;
    private double tagWidthPixels;
    private double distanceFeet;

    public AprilTagDistanceCalibration ( double screenWidthPixels, double screenHeightPixels, double tagWidthPixels, double distanceFeet ){
        this.screenWidthPixels = screenWidthPixels;
        this.screenHeightPixels = screenHeightPixels;
        this.tagWidthPixels = tagWidthPixels;
        this.distanceFeet = distanceFeet;
    }

    public double getScreenWidthPixels() {
        return screenWidthPixels;
    }
    
    public void setScreenWidthPixels(double screenWidthPixels) {
        this.screenWidthPixels = screenWidthPixels;
    }

    public double getScreenHeightPixels() {
        return screenHeightPixels;
    }
    
    public void setScreenHeightPixels(double screenHeightPixels) {
        this.screenHeightPixels = screenHeightPixels;
    }

    public double getTagWidthPixels() {
        return tagWidthPixels;
    }
    
    public void setTagWidthPixels(double tagWidthPixels) {
        this.tagWidthPixels = tagWidthPixels;
    }
    public double getDistanceFeet() {
        return distanceFeet;
    }
    
    public void setDistanceFeet(double distanceFeet) {
        this.distanceFeet = distanceFeet;
    }

}
