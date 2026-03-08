// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.LimelightHelpers;

// import edu.wpi.first.units.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.CANBus;

public final class Robot extends TimedRobot {
    public static final CANBus kCANBusGronk = new CANBus(Ports.CANBUS_DRIVE);
    public static final CANBus kCANBusJustice = new CANBus(Ports.CANBUS_OPS);
    public static final Pigeon2 pigeon = new Pigeon2(Ports.PIGEON, kCANBusGronk);
    private final RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    private boolean m_wasEnabledInTeleop = false;

    @Override
    public void robotPeriodic() {
        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-a", 4);

        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-b", 4);

        CommandScheduler.getInstance().run();

        NetworkTable tableA = NetworkTableInstance.getDefault().getTable("limelight-a");
        double[] poseA = tableA.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] bptsA = tableA.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double errorA = (Math.pow(bptsA[0], 2) + Math.pow(bptsA[1], 2) + Math.pow(bptsA[2], 2)) / 50;
        if (poseA[0] != 0 && errorA != 0) { // essentially this statement is true iff we have a measurement
            m_robotContainer.drivetrain.addVisionMeasurement(
                new Pose2d(poseA[0], poseA[1], new Rotation2d(poseA[5] * Math.PI / 180)),
                tableA.getEntry("ts_nt").getDouble(Double.NaN),
                MatBuilder.fill(Nat.N3(), Nat.N1(), errorA, errorA, 0.01)
            );
            System.out.println("Adding vision A: " + poseA[0] + ", " + poseA[1] + ", " + poseA[5] + ", +- " + errorA);
        }

        NetworkTable tableB = NetworkTableInstance.getDefault().getTable("limelight-b");
        double[] poseB = tableB.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] bptsB = tableB.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double errorB = (Math.pow(bptsB[0], 2) + Math.pow(bptsB[1], 2) + Math.pow(bptsB[2], 2)) / 50;
        if (poseB[0] != 0 && errorB != 0) { // essentially this statement is true iff we have a measurement
            m_robotContainer.drivetrain.addVisionMeasurement(
                new Pose2d(poseB[0], poseB[1], new Rotation2d(poseB[5] * Math.PI / 180)),
                tableB.getEntry("ts_nt").getDouble(Double.NaN),
                MatBuilder.fill(Nat.N3(), Nat.N1(), errorB, errorB, 0.01)
            );
            System.out.println("Adding vision B: " + poseB[0] + ", " + poseB[1] + ", " + poseB[5] + ", +- " + errorB);
        }
    }

    @Override
    public void disabledInit() {
        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-a", 1);

        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-b", 1);
    }

    @Override
    public void disabledPeriodic() {
        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-a", 1);

        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-b", 1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-a", 4);

        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-b", 4);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-a", 4);

        // LimelightHelpers.SetRobotOrientation(
        //     "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        // );
        // LimelightHelpers.SetIMUMode("limelight-b", 4);

        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
        m_robotContainer.climber.resetZerosAndTargetState();
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isEnabled()) {
            if (!m_wasEnabledInTeleop) {
                m_robotContainer.climber.resetZerosAndTargetState();
                m_wasEnabledInTeleop = true;
            }
        } else {
            m_wasEnabledInTeleop = false;
        }
        m_robotContainer.climber.move(
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.LEFT, RobotContainer.ControlBoard.CustomXboxController.Axis.Y),
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.LEFT, RobotContainer.ControlBoard.CustomXboxController.Axis.X),
            RobotContainer.controller.operator.getAxis(RobotContainer.ControlBoard.CustomXboxController.Side.RIGHT, RobotContainer.ControlBoard.CustomXboxController.Axis.Y)
        );
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
