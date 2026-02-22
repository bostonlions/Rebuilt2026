package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Robot.Ports;

import static frc.robot.Robot.kCANBus;
import static frc.robot.RobotContainer.ControlBoard.CustomXboxController.scaleWithDeadband;

public final class Climber implements edu.wpi.first.wpilibj2.command.Subsystem {
    private final double deadband = 0.25;
    private final double maxRPSupHook = 25;
    private final double maxRPSdownHook = 25;
    private final double maxRPSelevate = 25;
    private final TalonFX lowerHookMotor = new TalonFX(Ports.LOWER_HOOK_MOTOR, kCANBus);
    private final TalonFX upperHookMotor = new TalonFX(Ports.UPPER_HOOK_MOTOR, kCANBus);
    private final TalonFX elevatorMotor = new TalonFX(Ports.ELEVATOR_MOTOR, kCANBus);
    private final CANrange canRange = new CANrange(Ports.CANRANGE, kCANBus);

    public Climber() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().registerSubsystem(this);
        canRange.getConfigurator().apply(new CANrangeConfiguration());
        lowerHookMotor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.025)).withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(30).withMotionMagicJerk(1000).withMotionMagicCruiseVelocity(maxRPSdownHook + 1)));
        upperHookMotor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(0.025)).withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(30).withMotionMagicJerk(1000).withMotionMagicCruiseVelocity(maxRPSupHook + 1)));
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)).withSlot0(new Slot0Configs().withKP(0.025)).withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(300).withMotionMagicJerk(1000).withMotionMagicCruiseVelocity(maxRPSelevate + 1)));
    }

    public void move(double joyValueUpHook, double joyValueLowHook, double joyValueElevate) {
        lowerHookMotor.setControl(new MotionMagicVelocityDutyCycle(maxRPSdownHook * scaleWithDeadband(joyValueLowHook, deadband)));
        upperHookMotor.setControl(new MotionMagicVelocityDutyCycle(maxRPSupHook * scaleWithDeadband(joyValueUpHook, deadband)));
        elevatorMotor.setControl(new MotionMagicVelocityDutyCycle(maxRPSelevate * scaleWithDeadband(joyValueElevate, deadband)));
    }

    @Override
    public void periodic() {}
}
