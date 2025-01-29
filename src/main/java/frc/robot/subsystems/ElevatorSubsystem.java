package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

///import frc.robot.util.CANCoderUtil.CCUsage;
//import frc.robot.util.CANCoderUtil.CCCUsage
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

public class ElevatorSubsystem {
 private final SparkMax angleMotor;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder integratedAngleEncoder;
    public final SparkClosedLoopController angleController;

    private final SparkMax angleMotor2;
    private final SparkMaxConfig angleConfig2;
    private final RelativeEncoder integratedAngleEncoder2;
    public final SparkClosedLoopController angleController2;


    public ElevatorSubsystem(int angleMotorID){
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();
        angleMotor2 = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleConfig2 = new SparkMaxConfig();
        integratedAngleEncoder2 = angleMotor.getEncoder();
        configAngleMotor2();
        angleController2 = angleMotor.getClosedLoopController();





    }
private void configAngleMotor() {
       
       //2025 This is Deprecated angleMotor.restoreFactoryDefaults();
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kAll,angleConfig);
      
           angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);
                    //.setInverted(true);
            angleConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering moton the MAXSwerve Module.
                    //.inverted(true)
                    .positionConversionFactor(1/4) // radians
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

        
       angleConfig.voltageCompensation(12.0);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(2);
        integratedAngleEncoder.setPosition(0);

    }
private void configAngleMotor2() {
       
       //2025 This is Deprecated angleMotor.restoreFactoryDefaults();
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor2, Usage.kAll,angleConfig2);
      
           angleConfig2
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);
                    //.setInverted(true);
            angleConfig2.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering moton the MAXSwerve Module.
                    //.inverted(true)
                    .positionConversionFactor(1/4) // radians
                    .velocityConversionFactor(1); // radians per second
            angleConfig2.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1,1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0,1)
                    .minOutput(-1)
                    .maxOutput(1);
            angleConfig2.closedLoop.apply(angleConfig2.closedLoop);
            angleConfig2.apply(angleConfig2);

        
       angleConfig2.voltageCompensation(12.0);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor2.configure(angleConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(2);
        integratedAngleEncoder2.setPosition(0);

    }
 
    private void setAngle(Rotation2d angle) {
        angleController.setReference(angle.getRotations(), ControlType.kPosition);
        angleController2.setReference(-1*angle.getRotations(), ControlType.kPosition);
      
    }




}
