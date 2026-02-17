// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.LimelightHelpers;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;

public final class Robot extends TimedRobot {
    public static final Pigeon2 pigeon = new Pigeon2(Ports.PIGEON, Ports.CANBUS_DRIVE);
    public static final CANBus kCANBus = new CANBus(Ports.CANBUS_DRIVE, "./logs/example.hoot");
    private final RobotContainer m_robotContainer = new RobotContainer();
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay().withJoystickReplay();
    private Command m_autonomousCommand;

    @Override
    public void robotPeriodic() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-a", 4);

        LimelightHelpers.SetRobotOrientation(
            "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-b", 4);

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        NetworkTable tableA = NetworkTableInstance.getDefault().getTable("limelight-a");
        double[] poseA = tableA.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] bptsA = tableA.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double errorA = Math.sqrt(Math.pow(Math.pow(bptsA[0], 2) + Math.pow(bptsA[1], 2) + Math.pow(bptsA[2], 2), 2) / 10000);
        if (poseA[0] != 0) // essentially this statement is true iff we have a measurement
            m_robotContainer.drivetrain.addVisionMeasurement(
                new Pose2d(poseA[0], poseA[1], new Rotation2d(poseA[5])),
                tableA.getEntry("ts_nt").getDouble(Double.NaN),
                MatBuilder.fill(Nat.N3(), Nat.N1(), errorA, errorA, 0.001)
            );

        NetworkTable tableB = NetworkTableInstance.getDefault().getTable("limelight-b");
        double[] poseB = tableB.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] bptsB = tableB.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double errorB = Math.sqrt(Math.pow(Math.pow(bptsB[0], 2) + Math.pow(bptsB[1], 2) + Math.pow(bptsB[2], 2), 2) / 10000);
        if (poseB[0] != 0) // essentially this statement is true iff we have a measurement
            m_robotContainer.drivetrain.addVisionMeasurement(
                new Pose2d(poseB[0], poseB[1], new Rotation2d(poseB[5])),
                tableB.getEntry("ts_nt").getDouble(Double.NaN),
                MatBuilder.fill(Nat.N3(), Nat.N1(), errorB, errorB, 0.001)
            );
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-a", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-a", 1);

        LimelightHelpers.SetRobotOrientation(
            "limelight-b", pigeon.getYaw().getValue().in(Units.Degrees), 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetIMUMode("limelight-b", 1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    @Override
    public void teleopPeriodic() {}

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

    @Override
    public void simulationPeriodic() {}

    public static final class Ports {
        public static final String CANBUS_DRIVE = "Big Justice";
        public static final String CANBUS_OPS = "Baby Gronk";

        public static final int DRIVER_CONTROL = 0;
        public static final int OPERATOR_CONTROL = 1;

        public static final int PIGEON = 13;
        public static final int CANRANGE = 0;

        public static final int LAUNCHER = -1; // FIXME
        public static final int PITCH_MOTOR = -1; // FIXME
        public static final int PITCH_CANCODER = -1; //FIXME
        public static final int YAW_MOTOR = -1; // FIXME
        public static final int YAW_CANCODER_10 = -1; // FIXME
        public static final int YAW_CANCODER_11 = -1; // FIXME

        public static final int FL_DRIVE = 5;
        public static final int FL_ROTATION = 8;
        public static final int FL_CANCODER = 3;

        public static final int FR_DRIVE = 7;
        public static final int FR_ROTATION = 10;
        public static final int FR_CANCODER = 1;

        public static final int BL_DRIVE = 9;
        public static final int BL_ROTATION = 12;
        public static final int BL_CANCODER = 2;

        public static final int BR_DRIVE = 6;
        public static final int BR_ROTATION = 11;
        public static final int BR_CANCODER = 4;
    }
}
