package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;
import frc.robot.util.ReduceCANUsage.CANCoderUtil;
import frc.robot.util.ReduceCANUsage.CANCoderUtil.CCUsage;
import frc.robot.Robot;


import java.util.Map;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private int flipper=1;
    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    public CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    public final SparkClosedLoopController angleController;


    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    DRIVE_FF_S, DRIVE_FF_V, DRIVE_FF_A);

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
         angleEncoder = new CANcoder(canCoderID);
        configAngleEncoder();

        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();

        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();


        /* Drive Motoar Config */
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveConfig = new SparkMaxConfig();
        configDriveMotor();
        driveController = driveMotor.getClosedLoopController();

        lastAngle = getState().angle;

        Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);
    }
    
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void resetToAbsolute() {

       //TODO REMOVE THESE COMMENTS - Just Debug Code

       // System.out.println("Encoder "+moduleNumber+ " is set to Absolute Position");
       // System.out.println("The offset is "+ angleOffset.getRotations());
       // System.out.println("The Absolute Position is "+ angleEncoder.getAbsolutePosition().getValueAsDouble());
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble()-angleOffset.getRotations();
        if(absolutePosition<0){
            absolutePosition = 1+absolutePosition;
        }
    //    System.out.println("The Integrated encoder is reading: "+integratedAngleEncoder.getPosition());
        
        Timer.delay(2);

        integratedAngleEncoder.setPosition(Math.abs(absolutePosition));
        Timer.delay(2);

 //       System.out.println("Now the Integrated encoder is reading: "+integratedAngleEncoder.getPosition());


    }

        


    private void configAngleEncoder() {
         CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
         angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
       
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kAll,angleConfig);
      
           angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);
            angleConfig.encoder
                    .positionConversionFactor(1/ANGLE_GEAR_RATIO) // radians
                    .velocityConversionFactor(1); // radians per second
            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1,1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0,1)
                    .minOutput(-1)
                    .maxOutput(1);
            angleConfig.closedLoop.apply(angleConfig.closedLoop);
            angleConfig.apply(angleConfig);

        
            angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
            angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
            Timer.delay(2);
            resetToAbsolute();
    }

    private void configDriveMotor() {

        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(driveMotor, Usage.kAll,driveConfig);

        driveConfig.smartCurrentLimit(DRIVE_CURRENT_LIMIT) ;
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, DRIVE_PID_FF);

               driveConfig.voltageCompensation(VOLTAGE_COMPENSATION);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                     ClosedLoopSlot.kSlot0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;
                if (moduleNumber == 0){
            System.out.println("Angle Position Setting Mod" + moduleNumber + ": " + angle.getRotations());
        }
        angleController.setReference(angle.getRotations(), ControlType.kPosition);
    
      
        lastAngle = angle;
    }

    public void setAngleForX(double angle) {
        driveMotor.set(0);
    }

    public Rotation2d getAngle() {
       return Rotation2d.fromRotations(integratedAngleEncoder.getPosition());
    }
    

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
        }
   
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                Rotation2d.fromRotations(integratedAngleEncoder.getPosition())
                        );

    }

    }
