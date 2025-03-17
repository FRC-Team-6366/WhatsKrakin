package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SubsystemInput;
import frc.entech.subsystems.SubsystemOutput;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.pigeon.PigeonSubsystem;

public class SubsystemManager {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final PigeonSubsystem pigeonSubsystem = new PigeonSubsystem(); 
    public SubsystemManager() {
        driveSubsystem.initialize();
        pigeonSubsystem.initialize();

        periodic();
    }

    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystem;
    }

    public PigeonSubsystem getPigeonSubsystem() {
        return this.pigeonSubsystem;
    }

    public List<EntechSubsystem<? extends SubsystemInput, ? extends SubsystemOutput>> getSubsystemList() {
        ArrayList<EntechSubsystem<? extends SubsystemInput, ? extends SubsystemOutput>> r = new ArrayList<>();

        r.add(pigeonSubsystem);
        r.add(driveSubsystem);

        return r;
    }

    public final void periodic() {

    }
}
