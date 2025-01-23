package frc.robot.subsystems;


import static frc.robot.Constants.Swerve.ANGLE_CURRENT_LIMIT;
import static frc.robot.Constants.Swerve.ANGLE_GEAR_RATIO;
import static frc.robot.Constants.Swerve.ANGLE_IDLE_MODE;
import static frc.robot.Constants.Swerve.ANGLE_INVERT;
import static frc.robot.Constants.Swerve.ANGLE_PID_D;
import static frc.robot.Constants.Swerve.ANGLE_PID_FF;
import static frc.robot.Constants.Swerve.ANGLE_PID_I;
import static frc.robot.Constants.Swerve.ANGLE_PID_P;
import static frc.robot.Constants.Swerve.ANGLE_POSITION_CONVERSION_FACTOR;
import static frc.robot.Constants.Swerve.VOLTAGE_COMPENSATION;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.*;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;


public class ElevatorSubsytem {

    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;

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
            new SimpleMotorFeedforward(Constants.Elevator.d
                    DRIVE_FF_S, DRIVE_FF_V, DRIVE_FF_A);

    public ElevatorSubsytem(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
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

        lastAngle = getState().angle;

        Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //REMOVED MOD.KABS AS I THINK IT WAS DEPRECATED
        Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);
    }

    public enum Positions{
        LEVELFOUR(78),
        LEVELTHREE(76),
        LEVELTWO(96),
        LEVELONE(0)

        public final double position;
        Positions(double p) {
            this.position = p;
        }
        
    }
    public void setHeight(Positions p){
        setHeight(p.position);
    }


    angleController.setReference(angle.getRotations(), ControlType.kPosition);
   
    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;
    angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            angleConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering moton the MAXSwerve Module.
                    //.inverted(true)
                    .positionConversionFactor(1/ANGLE_GEAR_RATIO) // radians
                    .velocityConversionFactor(1); // radians per second
            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1, 0, 0)
                    .outputRange(0,1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0,1)
                    .minOutput(0)
                    .maxOutput(1);
            angleConfig.closedLoop.apply(angleConfig.closedLoop);
            angleConfig.apply(angleConfig);*/
    /* private void configAngleEncoder() {
        //angleEncoder.getConfigurator().apply(angleEncoder.getConfigurator().defaultCANCoderConfig);
         CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
         angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);*/


         /*public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not


        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);


        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop); */
        /* public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;


        /* Angle Encoder Config
         angleEncoder = new CANcoder(canCoderID);
        configAngleEncoder();


        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        //angleMotor.
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        //angleMotor.get
        //angleController.
        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();
*/
public void setAngleForX(double angle) {
        driveMotor.set(0);
        //angleMotor.set(TalonSRXControlMode.Position, (angle/360)*1023);
        angleMotor.set(.5);
        flipper= -1*flipper;
        //angleController.setPosition
        //angleController.setReference(angle, ControlType.kPosition);
       // angleController.
    }
 
public Rotation2d getAngle() {
        //SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
        //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        //return Rotation2d.fromDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees());
    } 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }
 
 //implement later
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

}



