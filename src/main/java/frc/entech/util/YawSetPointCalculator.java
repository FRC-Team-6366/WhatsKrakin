package frc.entech.util;
import edu.wpi.first.math.MathUtil;

// Calculate the yaw setpoint for auto alignment with AprilTag
// Algorithm assumes that as we get closer to the target, the yaw should get closer to the alignment value
// Closeness to the target is determined from the tag width (in pixels) ==> larger == closer
// We want the yaw to be at the desired value by the time the april tag width hits a magic number (FINAL_TAG_WIDTH_PX)

public class YawSetPointCalculator {

    private static final int FINAL_TAG_WIDTH_PX = 125;  // CAUTION: depends on camera resolution
    private final int initialTagWidthInPixel;
    private final double initialYaw;
    private final double finalYaw;
    private int currentTagWidthInPixels = 0;
    private boolean atFinal = false;

    public YawSetPointCalculator(int tagWidthInPixels, double currentYawAngle, double desiredYawAngle ) {
        this.initialTagWidthInPixel = tagWidthInPixels;
        this.currentTagWidthInPixels = tagWidthInPixels;
        this.initialYaw = currentYawAngle;
        this.finalYaw = desiredYawAngle;
        this.atFinal = false;
    }

    public double get(int tagWidthInPixels) {
        // Make sure tag width never decreases
        currentTagWidthInPixels = Math.max(tagWidthInPixels,currentTagWidthInPixels);
        currentTagWidthInPixels = Math.min(FINAL_TAG_WIDTH_PX,currentTagWidthInPixels);
        if (currentTagWidthInPixels >= FINAL_TAG_WIDTH_PX) {
            atFinal = true;
        }

        // cap ratio into the 0.0-1.0 range
        double ratio = (double)(FINAL_TAG_WIDTH_PX - currentTagWidthInPixels)/(double)(FINAL_TAG_WIDTH_PX - initialTagWidthInPixel);
        ratio = MathUtil.clamp(ratio*ratio,0.0,1.0);

        return (1.0-ratio)*finalYaw + ratio*initialYaw;
    }

    public boolean isReturningFinal() {
        return atFinal;
    }

}
