package frc.robot.subsystems;


import static frc.robot.Constants.Elevator.*;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.Positions;
import frc.robot.Constants;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;


public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMaxConfig leftMotorConfig;
    private final RelativeEncoder leftEncoder;
    private final SparkClosedLoopController leftController;

    private final SparkMax rightMotor;
    private final SparkMaxConfig rightMotorConfig;
    private final RelativeEncoder rightEncoder;
    private final SparkClosedLoopController rightController;
    private final TongueSubsystem Tongue;

    public ElevatorSubsystem(TongueSubsystem tongue) {
        leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        leftMotorConfig = new SparkMaxConfig();
        leftEncoder = leftMotor.getEncoder();
        leftController = leftMotor.getClosedLoopController();
        configLeftMotor();

        rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightMotorConfig = new SparkMaxConfig();
        rightEncoder = rightMotor.getEncoder();
        rightController = rightMotor.getClosedLoopController();
        configRightMotor();

        this.Tongue = tongue;
    }

    public void setPosition(Positions targetposition) {
        leftController.setReference(targetposition.position, ControlType.kPosition);
        rightController.setReference(targetposition.position, ControlType.kPosition);

        if (targetposition == Positions.LEVELTWO || targetposition == Positions.LEVELTHREE) {
            Tongue.retract();
        } else if (targetposition == Positions.LEVELONE || targetposition == Positions.LEVELFOUR) {
            Tongue.extend();
        } else {
            Tongue.retract();
        }
    }

    public void jogPositive(boolean b) {
        if (b) {
            leftMotor.set(.1);
            rightMotor.set(.1);
            System.out.println("Current Setting:" + leftEncoder.getPosition());
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    public void jogNegative(boolean b) {
        if (b) {
            leftMotor.set(-.1);
            rightMotor.set(-.1);
            System.out.println("Current Setting:" + leftEncoder.getPosition());
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    private void configLeftMotor() {
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(leftMotor, Usage.kPositionOnly, leftMotorConfig);
        leftMotorConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT);
        leftMotorConfig.inverted(ANGLE_INVERT);
        leftMotorConfig.idleMode(ANGLE_IDLE_MODE);
        //angleConfig.encoder.positionConversionFactor(1/ANGLE_POSITION_CONVERSION_FACTOR);
        leftMotorConfig.encoder.positionConversionFactor(1.0 / Constants.Elevator.ANGLE_POSITION_CONVERSION_FACTOR);
        leftMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.Elevator.P, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(false)
                .positionWrappingInputRange(0, 1)
                .minOutput(-1)
                .maxOutput(1);
        leftMotorConfig.closedLoop.apply(leftMotorConfig.closedLoop);
        leftMotorConfig.apply(leftMotorConfig);
        leftMotorConfig.voltageCompensation(VOLTAGE_COMPENSATION);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder.setPosition(0);
    }

    private void configRightMotor() {
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(rightMotor, Usage.kPositionOnly, rightMotorConfig);
        rightMotorConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT);

        rightMotorConfig.inverted(ANGLE_INVERT_2);

        rightMotorConfig.idleMode(ANGLE_IDLE_MODE);

        // angleConfig2.encoder.positionConversionFactor(1/ANGLE_POSITION_CONVERSION_FACTOR);
        rightMotorConfig.encoder.positionConversionFactor(1.0 / ANGLE_POSITION_CONVERSION_FACTOR);

        rightMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.Elevator.P, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(false)
                .positionWrappingInputRange(0, 1)
                .minOutput(-1)
                .maxOutput(1);
        rightMotorConfig.closedLoop.apply(rightMotorConfig.closedLoop);
        rightMotorConfig.apply(rightMotorConfig);
        rightMotorConfig.voltageCompensation(VOLTAGE_COMPENSATION);

        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightEncoder.setPosition(0);

    }
}




