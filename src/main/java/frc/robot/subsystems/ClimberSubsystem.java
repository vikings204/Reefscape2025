package frc.robot.subsystems;

import static frc.robot.Constants.Arm.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    public ClimberSubsystem() {
        motor = new SparkMax(43, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        encoder = motor.getEncoder();
        configMotor();
        controller = motor.getClosedLoopController();
    }

    public void ShootArm(boolean b) {
        if (b) {
            controller.setReference(encoder.getPosition() + .1, ControlType.kPosition);
        } else {
            controller.setReference(encoder.getPosition(), ControlType.kPosition);
        }
    }

    public void NegativeShootArm(boolean b) {
        if (b) {
            controller.setReference(encoder.getPosition() - .1, ControlType.kPosition);
        } else {
            controller.setReference(encoder.getPosition(), ControlType.kPosition);
        }
    }

    private void configMotor() {
        motorConfig
                .idleMode(ANGLE_IDLE_MODE)
                .smartCurrentLimit(40)
                .inverted(false);
        //.setInverted(true);
        motorConfig.encoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering moton the MAXSwerve Module.
                //.inverted(true)
                .positionConversionFactor(1.0 / 100.0) // radians
                .velocityConversionFactor(1); // radians per second
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(10, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(false)
                .positionWrappingInputRange(0, 1)
                .minOutput(-1)
                .maxOutput(1);
        motorConfig.closedLoop.apply(motorConfig.closedLoop);
        motorConfig.apply(motorConfig);
        System.out.println("I CONFIGURED THE CLIMBER MOTOR");

        motorConfig.voltageCompensation(VOLTAGE_COMPENSATION);
        //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //angleMotor.burnFlash();
        Timer.delay(2);
        encoder.setPosition(0);

    }
}
