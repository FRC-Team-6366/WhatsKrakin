package frc.entech.util;

public class StoppingCounter {

  private double targetCounts = 0;
  private double currentCounts = 0;

  public StoppingCounter(double targetCounts) {
    this.targetCounts = targetCounts;
  }

  public boolean isFinished(boolean conditionToCheck) {
    if (conditionToCheck) {
      currentCounts++;
    } else {
      currentCounts = 0;
    }
    return currentCounts > EntechUtils.getLoopsPerSecond(targetCounts);
  }

  public void reset() {
    this.currentCounts = 0;
  }
}
