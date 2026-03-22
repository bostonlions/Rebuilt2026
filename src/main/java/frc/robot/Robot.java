// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.LimelightHelpers;
import frc.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.launcher.Launcher;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

public final class Robot extends TimedRobot {
    public static final CANBus kCANBusGronk = new CANBus(Ports.CANBUS_DRIVE);
    public static final CANBus kCANBusJustice = new CANBus(Ports.CANBUS_OPS);
    public static final Pigeon2 pigeon = new Pigeon2(Ports.PIGEON, kCANBusGronk);
    private static final Timer clock = new Timer();
    private final RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    private boolean m_wasEnabledInTeleop = false;
    private int IMUmode = 1;
    private final boolean useVision = false;

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (!useVision) return;

        final double yaw = pigeon.getYaw().getValue().in(Units.Degrees);

        LimelightHelpers.SetRobotOrientation("limelight-a", yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode("limelight-a", IMUmode);

        LimelightHelpers.SetRobotOrientation("limelight-b", yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode("limelight-b", IMUmode);

        PoseEstimate bpa = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
        boolean aValid = bpa != null && bpa.rawFiducials != null && bpa.rawFiducials.length != 0;
        PoseEstimate bpb = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
        boolean bValid = bpb != null && bpb.rawFiducials != null && bpb.rawFiducials.length != 0;
        PoseEstimate bp; // the pose estimate to actually use; determined later

        if (aValid || bValid) {
            if (!aValid) bp = bpb;
            else if (!bValid) bp = bpa;
            else if (bpa.rawFiducials[0].distToCamera > bpa.rawFiducials[0].distToCamera) bp = bpb;
            else bp = bpa;

            System.out.println("id: " + bp.rawFiducials[0].id + "; limelight-" + (bp == bpa ? "a" : "b"));

            double error = Math.pow(bp.rawFiducials[0].distToCamera, 2) / 50;
            m_robotContainer.drivetrain.addVisionMeasurement(bp.pose, bp.timestampSeconds,
                MatBuilder.fill(Nat.N3(), Nat.N1(), error, error, 0.01));
        }

        if (aValid && bValid) {
            System.out.print("dx: " + Math.abs(bpa.pose.getX() - bpb.pose.getX()));
            System.out.print("; dy: " + Math.abs(bpa.pose.getY() - bpb.pose.getY()));
            System.out.println("; dt: " + Math.abs(bpa.pose.getRotation().getDegrees() - bpb.pose.getRotation().getDegrees()));
        }
    }

    @Override
    public void disabledInit() {
        IMUmode = 1;
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        IMUmode = 4;

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        Launcher.getInstance().forcePitchDown();
        
        IMUmode = 4;

        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
        m_robotContainer.climber.resetZerosAndTargetState();

        if (clock.isRunning()) clock.reset(); else clock.start();
        CommandScheduler.getInstance().schedule(new WaitCommand(10).andThen(() -> clock.reset()));
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isEnabled()) {
            if (!m_wasEnabledInTeleop) {
                m_robotContainer.climber.resetZerosAndTargetState();
                m_wasEnabledInTeleop = true;
            }
            if (clock.hasElapsed(25)) clock.reset();
        } else m_wasEnabledInTeleop = false;

        m_robotContainer.climber.move(
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.LEFT, RobotContainer.ControlBoard.CustomXboxController.Axis.Y),
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.LEFT, RobotContainer.ControlBoard.CustomXboxController.Axis.X),
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.RIGHT, RobotContainer.ControlBoard.CustomXboxController.Axis.Y)
        );
    }

    /** Gets the number of seconds left of hopper being for current color */
    public static double getCountDown() {
        return ((double) Math.round((25 - clock.get()) * 10)) / 10;
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    public static final class Ports {
        public static final String CANBUS_DRIVE = "Baby Gronk";
        public static final String CANBUS_OPS = "Big Justice";

        /** Driver = 0, Operator = 1 (USB/DH port) */
        public static final int DRIVER_CONTROL = 0;
        public static final int OPERATOR_CONTROL = 1;

        /** Climber: lower hook, upper hook, elevator */
        public static final int LOWER_HOOK_MOTOR = 33;
        public static final int UPPER_HOOK_MOTOR = 34;
        public static final int ELEVATOR_MOTOR = 35;
        public static final int LOWER_HOOK_CANCODER = 32;

        /** Launcher: main flywheel */
        public static final int LAUNCHER = 49;
        public static final int PITCH_MOTOR = 50; 
        public static final int PITCH_CANCODER = 38; 
        public static final int YAW_MOTOR = 40;
        public static final int YAW_CANCODER_11 = 52;
        public static final int YAW_CANCODER_12 = 54;
        /** Feeder (indexer) motors */
        public static final int FEEDER_SPINNER = 41;
        public static final int FEEDER_ROLLER = 42;

        /** Intake: extend and spin */
        public static final int INTAKE_EXTEND = 36;
        public static final int INTAKE_SPIN = 44;

        /** CANrange time-of-flight proximity sensor */
        public static final int CANRANGE = 51;

        // ----- CANBUS_DRIVE (Big Justice) -----
        public static final int PIGEON = 13;

        /** Front Left swerve module */
        public static final int FL_DRIVE = 5;
        public static final int FL_ROTATION = 8;
        public static final int FL_CANCODER = 3;

        /** Front Right */
        public static final int FR_DRIVE = 7;
        public static final int FR_ROTATION = 10;
        public static final int FR_CANCODER = 1;

        /** Back Left */
        public static final int BL_DRIVE = 9;
        public static final int BL_ROTATION = 12;
        public static final int BL_CANCODER = 2;

        /** Back Right */
        public static final int BR_DRIVE = 6;
        public static final int BR_ROTATION = 11;
        public static final int BR_CANCODER = 4;
    }
}
