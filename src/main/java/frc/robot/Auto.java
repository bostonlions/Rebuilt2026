package frc.robot;

import java.util.Arrays;
import java.util.Map;
import static java.util.Map.entry;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Trimmer;

public final class Auto extends SubsystemBase implements Subsystem {
    private static Auto instance = null;
    private static final Drivetrain drive = Drivetrain.getInstance();
    private static final Climber climber = Climber.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Launcher launcher = Launcher.getInstance();
    private static final Trimmer trimmer = Trimmer.getInstance();
    private static final Map<String, Command> commands = Map.ofEntries(
        entry("00 - None", debug(() -> "Autonomous started with no command chosen")),
        entry("01 - Drive 1m forward", Commands.sequence(
            drive.runOnce(() -> drive.seedFieldCentric(Rotation2d.kZero)),
            drive.applyRequest(() -> SwerveConstants.drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)).withTimeout(2.0),
            drive.applyRequest(() -> SwerveConstants.idle)
        ))
    );

    
    public static Auto getInstance() {
        if (instance == null) instance = new Auto();
        return instance;
    }

   
    private volatile int commandIdx = 0;

    private String[] commandNames;
    private String allCommands;

    /** NetworkTables entries for Elastic Dashboard auto selector (Trimmer/Autonomous). */
    private final NetworkTableEntry ntSelected;
    private final NetworkTableEntry ntOptions;

    private Auto() {
        commandNames = commands.keySet().toArray(new String[0]);
        Arrays.sort(commandNames);
        allCommands = String.join("\n", commandNames);
        initTrimmer();

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Trimmer").getSubTable("Autonomous");
        table.getEntry("type").setString("String Chooser");
        ntSelected = table.getEntry("selected");
        ntOptions = table.getEntry("options");
        ntOptions.setStringArray(commandNames);
        ntSelected.setString(commandNames[commandIdx]);
        /*NetworkTableInstance.getDefault().addListener(
            ntSelected,
            java.util.EnumSet.of(edu.wpi.first.networktables.NetworkTableEvent.Kind.kValueAll, edu.wpi.first.networktables.NetworkTableEvent.Kind)
        );*/
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
        ntSelected.setString(commandNames[commandIdx]);
    }

    /** Call from robotPeriodic to sync selection to Elastic Dashboard. */
    public void publishToDashboard() {
        ntSelected.setString(commandNames[commandIdx]);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Auton");
        builder.addStringProperty("Command", () -> commandNames[commandIdx], null);
        builder.addStringProperty("Commands", () -> allCommands, null);
    }

    private void initTrimmer() {
        Trimmer.getInstance().add("Autonomous", "Command", () -> (double) commandIdx, this::inc);
    }

}
