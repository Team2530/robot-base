package frc.robot.util.homebrew;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class Flagpole {
    private ArrayList<Command> flags = new ArrayList<>();

    public InstantCommand raiseFlaggedCommand(Supplier<Command> supplier) {
        return new InstantCommand(() -> {
            flags.add(
                    supplier.get()
                    );
        });
    }


    public Command catchFlags() {
        Command command = new ParallelRaceGroup(
            flags.toArray(
                new Command[flags.size()]
            )
        );

        flags.clear();

        return command;
    }
}
