package frc.entech.commands;

public class AutonomousException extends RuntimeException {
    public AutonomousException(String message) {
        super(message);
    }
    public AutonomousException(String message, Exception e) {
        super(message, e);
    }
}
