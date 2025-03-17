package frc.robot.processors.filters;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.drive.DriveInput;

public class MaxConstraintFilter implements DriveFilterI {
        private static final double MAX_SPEED_PERCENT = 1;
        private static final double DEADBAND_SPEED_PERCENT = 0.1;

        @Override
        public DriveInput process(DriveInput input) {
                DriveInput processedInput = new DriveInput(input);

                processedInput.setXSpeed(MathUtil.applyDeadband(
                                MathUtil.clamp(input.getXSpeed(), -MAX_SPEED_PERCENT,
                                                MAX_SPEED_PERCENT),
                                DEADBAND_SPEED_PERCENT));
                processedInput.setYSpeed(MathUtil.applyDeadband(
                                MathUtil.clamp(input.getYSpeed(), -MAX_SPEED_PERCENT,
                                                MAX_SPEED_PERCENT),
                                DEADBAND_SPEED_PERCENT));
                processedInput.setRotation(MathUtil.applyDeadband(
                                MathUtil.clamp(input.getRotation(), -MAX_SPEED_PERCENT,
                                                MAX_SPEED_PERCENT),
                                DEADBAND_SPEED_PERCENT));

                return processedInput;
        }

}
