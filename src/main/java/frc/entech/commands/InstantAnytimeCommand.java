package frc.entech.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InstantAnytimeCommand extends InstantCommand {
    public InstantAnytimeCommand(Runnable toRun) {
        super(toRun);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
