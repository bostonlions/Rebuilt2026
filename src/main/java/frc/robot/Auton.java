package frc.robot;

import java.util.Arrays;
import java.util.Map;

import choreo.auto.AutoFactory;

import static java.util.Map.entry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trimmer;
import frc.robot.subsystems.launcher.Launcher;

public final class Auton extends SubsystemBase {
    private static Auton instance = null;
    private static final Launcher launcher = Launcher.getInstance();
    private static final Climber climber = Climber.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final AutoFactory autoFactory = RobotContainer.autoFactory;
    private static final Map<String, Command> commands = Map.ofEntries(
        entry("0: None", new PrintCommand("Autonomous started with no command chosen")),
        entry("1: Shoot from left corner of hub", Commands.sequence(
            new InstantCommand(() -> launcher.simpleToggle(1475, 90-15, -45)),
            sleep(4), new InstantCommand(() -> intake.setSpinner(true), intake), sleep(4),
            new InstantCommand(() -> intake.setSpinner(false), intake), sleep(2),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("2: Shoot from right corner of hub", Commands.sequence(
            new InstantCommand(() -> launcher.simpleToggle(1475, 90-15, 45)),
            sleep(4), new InstantCommand(() -> intake.setSpinner(true), intake), sleep(4),
            new InstantCommand(() -> intake.setSpinner(false), intake), sleep(2),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("3: Test path", Commands.sequence(
            
            followPathCommand("Testing")
        )),
        entry("4: Climb From Right", Commands.sequence(
            new InstantCommand(() -> launcher.simpleToggle(1475, 90-15, 45)),
            sleep(4), new InstantCommand(() -> intake.setSpinner(true), intake), sleep(4),
            new InstantCommand(() -> intake.setSpinner(false), intake), sleep(2),
            new InstantCommand(() -> launcher.simpleToggle()),
            followPathCommand("ClimbFromRightSetup"),
            climber.elevatorUp(),
            followPathCommand("ClimbFromRightDriveIn"),
            climber.elevatorDown()
        )),
        entry("5: Mid and Back", new ParallelCommandGroup(
            followPathCommand("GoToMidAndBack"),
            new WaitCommand(1).andThen(
                new InstantCommand(() -> intake.setExtension(true), intake),
                new InstantCommand(() -> intake.toggleSpin(), intake),
                new WaitCommand(2),
                new InstantCommand(() -> intake.toggleSpin(), intake),
                new InstantCommand(() -> intake.setExtension(false), intake)
            )
        ).andThen(() -> launcher.setMode(Launcher.Mode.FIRE), launcher))
    );

    private static Command followPathCommand(String path) {
        return autoFactory.resetOdometry(path).andThen(autoFactory.trajectoryCmd(path));
    }
    
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
