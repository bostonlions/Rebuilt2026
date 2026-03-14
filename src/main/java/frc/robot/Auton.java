package frc.robot;

import java.util.Arrays;
import java.util.Map;

import static java.util.Map.entry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Trimmer;

public final class Auton extends SubsystemBase {
    private static Auton instance = null;
    private static final Launcher launcher = Launcher.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Map<String, Command> commands = Map.ofEntries(
        entry("0: None", new PrintCommand("Autonomous started with no command chosen")),
        entry("1: Shoot from left corner of hub", Commands.sequence(
            new InstantCommand(() -> launcher.simpleToggle(1475, 15, -45)),
            sleep(4), new InstantCommand(() -> intake.setSpinner(true), intake), sleep(4),
            new InstantCommand(() -> intake.setSpinner(false), intake), sleep(2),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("2: Shoot from right corner of hub", Commands.sequence(
            new InstantCommand(() -> launcher.simpleToggle(1475, 15, 45)),
            sleep(4), new InstantCommand(() -> intake.setSpinner(true), intake), sleep(4),
            new InstantCommand(() -> intake.setSpinner(false), intake), sleep(2),
            new InstantCommand(() -> launcher.simpleToggle())
        ))
        // entry("3: Shoot from far left", Commands.sequence(
        //     // Shoot for 10 sec, low speed and high angle, then turn off
        //     new InstantCommand(() -> launcher.simpleToggle(1800, 25, -80)),
        //     sleep(10),
        //     new InstantCommand(() -> launcher.simpleToggle())
        // )),
        // entry("4: Shoot from far right", Commands.sequence(
        //     // Shoot for 10 sec, low speed and high angle, then turn off
        //     new InstantCommand(() -> launcher.simpleToggle(1800, 25, 80)),
        //     sleep(10),
        //     new InstantCommand(() -> launcher.simpleToggle())
        // )),
        // entry("3: Shoot straight default", Commands.sequence(
        //     // Shoot for 10 sec, default speed and angle, then turn off
        //     new InstantCommand(() -> launcher.simpleToggle()),
        //     sleep(10),
        //     new InstantCommand(() -> launcher.simpleToggle())
        // ))
        // entry("4: Shoot and climb: left hub corner", Commands.sequence(
        //     new InstantCommand(() -> launcher.simpleToggle(1475, 15, -45)),
        //     sleep(8),
        //     new InstantCommand(() -> launcher.simpleToggle())
        //     // parallel: drive to 1 foot in front of tower, while prepping to climb
        //     // then slowly drive forward too far (2 feet?) so we know we hit the tower
        //     // then climb
        // )),
        // entry("5: Shoot and climb: right hub corner", Commands.sequence(
        //     new InstantCommand(() -> launcher.simpleToggle(1475, 15, 45)),
        //     sleep(8),
        //     new InstantCommand(() -> launcher.simpleToggle())
        // ))
    );
    
    public static Auton getInstance() {
        if (instance == null) instance = new Auton();
        return instance;
    }
   
    private volatile int commandIdx = 0;

    private String[] commandNames;
    private String allCommands;

    private Auton() {
        commandNames = commands.keySet().toArray(new String[0]);
        Arrays.sort(commandNames);
        allCommands = String.join("\n", commandNames);
        initTrimmer();
    }

    private static Command sleep(double secs) {
        return new WaitCommand(secs);
    }

    public Command getCommand() {
        return commands.get(commandNames[commandIdx]);
    }

    private void inc(boolean up) {
        commandIdx = (commandIdx + commands.size() + (up ? 1 : -1)) % commands.size();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Auton");
        builder.addStringProperty("Command", () -> commandNames[commandIdx], null);
        builder.addStringProperty("Commands", () -> allCommands, null);
    }

    private void initTrimmer() {
        Trimmer.getInstance().add("Auton", "Command", () -> (double) commandIdx, this::inc);
    }

}
