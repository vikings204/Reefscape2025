package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

///import frc.robot.util.CANCoderUtil.CCUsage;
//import frc.robot.util.CANCoderUtil.CCCUsage
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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

    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;

    //public final TalonSRX angleMotor;
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;
    //private final TalonSRXFeedbackDevice angleController;

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
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();


        /* Drive Motor Config */
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveConfig = new SparkMaxConfig();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;

        Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //REMOVED MOD.KABS AS I THINK IT WAS DEPRECATED
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
        //angleMotor.getAnalog().setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        
        // 2025 UPDATED TO USE CANCODER AND SET POSITION ABS POSITION - NEED TO WORK ON TO FINALIZE
        //MIGHT HAVE TO CHANGE THIS BACK TO JUST GET THE ROBOT UP AND RUNNING!
        
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        System.out.println("Encoder" +moduleNumber+ "Absolute Position: "+absolutePosition);    
        System.out.println("Encoder "+moduleNumber+ " is Zerod");
        
        
        integratedAngleEncoder.setPosition(absolutePosition);
    }

        public void resetToAbsoluteTest() {
        //angleMotor.getAnalog().setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
       
        double absolutePosition = convertToDegrees(angleMotor.getAnalog().getVoltage());
        System.out.println("Encoder" +moduleNumber+ "Absolute Position: "+absolutePosition);    
       
        integratedAngleEncoder.setPosition(absolutePosition);
         System.out.println("Encoder "+moduleNumber+ " is set to postion: "+integratedAngleEncoder.getPosition());
    }
    public double convertToDegrees(double voltage){
        return (voltage/3.3)*360 - angleOffset.getDegrees();
    }

    //JOE EDIT THIS

    private void configAngleEncoder() {
        //angleEncoder.getConfigurator().apply(angleEncoder.getConfigurator().defaultCANCoderConfig);
         CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
         angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
         
         //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);

        //angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    private void configAngleMotor() {
       
       //2025 This is Deprecated angleMotor.restoreFactoryDefaults();
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly,angleConfig);
       angleConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT) ;
       // replaced above angleMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
       angleConfig.inverted(ANGLE_INVERT); 
       // replaced above angleMotor.setInverted(ANGLE_INVERT);
       angleConfig.idleMode(ANGLE_IDLE_MODE); 
       // replaced above angleMotor.setIdleMode(ANGLE_IDLE_MODE);
        angleConfig.encoder.positionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        // replaced above integratedAngleEncoder.setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ANGLE_PID_P, ANGLE_PID_I, ANGLE_PID_D, ANGLE_PID_FF)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, ANGLE_POSITION_CONVERSION_FACTOR);
        /* Replaced above 
        angleController.setP(ANGLE_PID_P);
        angleController.setI(ANGLE_PID_I);
        angleController.setD(ANGLE_PID_D);
        angleController.setFF(ANGLE_PID_FF);*/
       // angleController.setFeedbackDevice(angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute));
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(2);
        resetToAbsolute();
    }

    private void configDriveMotor() {

        //driveMotor.restoreFactoryDefaults();
        // ASK NEEV ReduceCANUsage.SparkMax.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(driveMotor, Usage.kAll,driveConfig);

        driveConfig.smartCurrentLimit(DRIVE_CURRENT_LIMIT) ;
        // replaced above driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        //driveMotor.setInverted(DRIVE_INVERT);
        // Both replaced above driveMotor.setIdleMode(DRIVE_IDLE_MODE);
        driveConfig.encoder
            .positionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        // REPLACED ABOVE driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        //driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, DRIVE_PID_FF);
       /* REPLACE ABOVE
         driveController.setP(DRIVE_PID_P);
        driveController.setI(DRIVE_PID_I);
        driveController.setD(DRIVE_PID_D);
        driveController.setFF(DRIVE_PID_FF);
        */
        driveConfig.voltageCompensation(VOLTAGE_COMPENSATION);

        // replaced above driveMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        driveMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //replaced above driveMotor.burnFlash();
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
            /*SmartDashboard.putNumber("Angle Position Setting Mod" + moduleNumber, angle.getDegrees());
            SmartDashboard.putNumber("Encoder Position Setting without Offset" + moduleNumber, (((angle.getDegrees())/360)*1023));   
            SmartDashboard.putNumber("Encoder Position Setting with Offset" + moduleNumber, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023)); */
       // double targetVoltage = angle.getDegrees()- angleOffset.getDegrees();
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void setAngleForX(double angle) {
        driveMotor.set(0);
        //angleMotor.set(TalonSRXControlMode.Position, (angle/360)*1023);
        angleController.setReference(angle, ControlType.kPosition);
    }

    private Rotation2d getAngle() {
        //SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
        //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        //return Rotation2d.fromDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }
    

    public Rotation2d getCanCoder() {
        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
       //return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
       return Rotation2d.fromDegrees(360.0*angleEncoder.getAbsolutePosition().getValueAsDouble());

       //return Rotation2d.fromDegrees((angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute).getVoltage()/3.3)*360);
    }
   
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public void resetEncoder() {
        integratedAngleEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        //SmartDashboard.putNumber("Raw Angle Reading " + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360);
        //System.out.println("Encoder Position: "+((angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees()));
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                //Rotation2d.fromDegrees(getCanCoder().getDegrees()- angleOffset.getDegrees())
                Rotation2d.fromDegrees(integratedAngleEncoder.getPosition())
                
                /////////////
                //NEW CODE//
                ///////////
                
                //Rotation2d.fromDegrees(angleEncoder.getPosition().getValueAsDouble() - angleOffset.getDegrees())
        );

    }

    //public Rotation2d getAbsoluteAnglePosition() {
      //  return new Rotation2d(angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute).getVoltage());

    //}
}
