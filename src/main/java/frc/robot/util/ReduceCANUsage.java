package frc.robot.util;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

public class ReduceCANUsage {
    public static class SparkMax {
        // adapted from https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        public enum Usage {
            kAll,
            kPositionOnly,
            kVelocityOnly,
            kMinimal
        }

        public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage, SparkMaxConfig config/*, boolean enableFollowing*/) {
            if (/*enableFollowing*/DriverStation.isEStopped()) {
                config.signals.appliedOutputPeriodMs(10);

               // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
            } else {
                config.signals.appliedOutputPeriodMs(500);

                //motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
            }

            if (usage == Usage.kAll) {
                config.signals.motorTemperaturePeriodMs(20);
                config.signals.busVoltagePeriodMs(20);
                config.signals.primaryEncoderVelocityPeriodMs(20);
                config.signals.outputCurrentPeriodMs(20);
                
                config.signals.primaryEncoderPositionPeriodMs(20);

                config.signals.analogPositionPeriodMs(50);
                config.signals.analogVoltagePeriodMs(50);
                config.signals.analogVelocityPeriodMs(50);

    
                /* 
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);*/
            } else if (usage == Usage.kPositionOnly) {
                config.signals.motorTemperaturePeriodMs(500);
                config.signals.busVoltagePeriodMs(500);
                config.signals.primaryEncoderVelocityPeriodMs(500);
                config.signals.outputCurrentPeriodMs(500);
                
                config.signals.primaryEncoderPositionPeriodMs(20);

                config.signals.analogPositionPeriodMs(500);
                config.signals.analogVoltagePeriodMs(500);
                config.signals.analogVelocityPeriodMs(500);
                /* 
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);*/
            } else if (usage == Usage.kVelocityOnly) {
                config.signals.motorTemperaturePeriodMs(20);
                config.signals.busVoltagePeriodMs(20);
                config.signals.primaryEncoderVelocityPeriodMs(20);
                config.signals.outputCurrentPeriodMs(20);
                
                config.signals.primaryEncoderPositionPeriodMs(500);

                config.signals.analogPositionPeriodMs(500);
                config.signals.analogVoltagePeriodMs(500);
                config.signals.analogVelocityPeriodMs(500);
                /* 
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);*/
            } else if (usage == Usage.kMinimal) {
                config.signals.motorTemperaturePeriodMs(20);
                config.signals.busVoltagePeriodMs(20);
                config.signals.primaryEncoderVelocityPeriodMs(20);
                config.signals.outputCurrentPeriodMs(20);
                
                config.signals.primaryEncoderPositionPeriodMs(20);

                config.signals.analogPositionPeriodMs(50);
                config.signals.analogVoltagePeriodMs(50);
                config.signals.analogVelocityPeriodMs(50);
                /*
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);*/
            }
        }
    }
}

