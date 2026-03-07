package frc.robot;

import java.util.Arrays;
import java.util.Map;
import static java.util.Map.entry;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Trimmer;

public final class Auton extends SubsystemBase {
    private static Auton instance = null;
    private static final Drivetrain drivetrain = Drivetrain.getInstance();
    private static final Climber climber = Climber.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Launcher launcher = Launcher.getInstance();
    private static final Map<String, Command> commands = Map.ofEntries(
        entry("00 - None", debug(() -> "Autonomous started with no command chosen")),
        entry("01 - Shoot from corner of hub", Commands.sequence(
            // Shoot for 10 sec, low speed and high angle, then turn off
            new InstantCommand(() -> launcher.simpleToggle(1475, 15, -45)),
            sleep(10),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("02 - Shoot from far left", Commands.sequence(
            // Shoot for 10 sec, low speed and high angle, then turn off
            new InstantCommand(() -> launcher.simpleToggle(1800, 25, -80)),
            sleep(10),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("03 - Shoot from far right", Commands.sequence(
            // Shoot for 10 sec, low speed and high angle, then turn off
            new InstantCommand(() -> launcher.simpleToggle(1800, 25, 80)),
            sleep(10),
            new InstantCommand(() -> launcher.simpleToggle())
        )),
        entry("04 - Shoot straight default", Commands.sequence(
            // Shoot for 10 sec, default speed and angle, then turn off
            new InstantCommand(() -> launcher.simpleToggle()),
            sleep(10),
            new InstantCommand(() -> launcher.simpleToggle())
        ))
       /*  entry("01 - Drive 1m forward", Commands.sequence(
            This bugs out the drive. I need to talk with whoever did the drivetrain about what to do here.
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() -> SwerveConstants.drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)).withTimeout(2.0),
            drivetrain.applyRequest(() -> SwerveConstants.idle)
        ))*/
        //entry(null, null)
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

    private static Command debug(Supplier<String> s) {
        return new InstantCommand(() -> System.out.println(s.get()));
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
