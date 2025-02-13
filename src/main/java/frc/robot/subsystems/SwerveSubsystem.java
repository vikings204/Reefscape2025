
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.Swerve.*;


public class SwerveSubsystem extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(PIGEON2_ID, "rio");
    //private final SwerveDriveOdometry swerveOdometry; // Odometry class helps track where the robot is relative to where it started
    public final SwerveModule[] modules; // Array of the 4 swerve modules

    public SwerveSubsystem() {
        var toApply = new Pigeon2Configuration();
        gyro.getConfigurator().apply(toApply);

        zeroGyro();
        //m_gyro.calibrate();
        modules = new SwerveModule[]{
                        new SwerveModule(0, Mod0.DRIVE_MOTOR_ID, Mod0.ANGLE_MOTOR_ID, Mod0.ANGLE_OFFSET, Mod0.CAN_CODER_ID), //Each Constant set is specific to a motor pair
                        new SwerveModule(1, Mod1.DRIVE_MOTOR_ID, Mod1.ANGLE_MOTOR_ID, Mod1.ANGLE_OFFSET, Mod1.CAN_CODER_ID),
                        new SwerveModule(2, Mod2.DRIVE_MOTOR_ID, Mod2.ANGLE_MOTOR_ID, Mod2.ANGLE_OFFSET, Mod2.CAN_CODER_ID),
                        new SwerveModule(3, Mod3.DRIVE_MOTOR_ID, Mod3.ANGLE_MOTOR_ID, Mod3.ANGLE_OFFSET, Mod3.CAN_CODER_ID)
                };

       
        for (SwerveModule mod : modules) {
            Shuffleboard.getTab("swerve").addNumber("position: module " + mod.moduleNumber, () -> mod.getPosition().distanceMeters);
            Shuffleboard.getTab("swerve").addNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
        }
        Shuffleboard.getTab("main").addNumber("gyro angle", () -> {
            Rotation2d yaw = getYaw();
            return yaw.getDegrees();
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) :
                        new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        //System.out.println("Current Heading: "+getYaw());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
        }
    }


    public ChassisSpeeds getSpeeds() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getStates());
    }


    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }
    public void setX(){
        for (SwerveModule mod : modules) {
            mod.resetToAbsolute();
           // System.out.println("Current Mod: "+mod.moduleNumber);
        }
    }

    public void setGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (GYRO_INVERT)
                ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

public void resetEncoders(){
    for (int i = 0; i<4; i++){
        modules[i].resetToAbsolute();
      }
}

    @Override
    public void periodic() {
        //swerveOdometry.update(getYaw(), getPositions());
        //field.setRobotPose(getPose());

        // for (SwerveModule mod : modules) {
        //     //  SmartDashboard.putNumber(
        //     //    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180)); // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        //     Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond); //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }
    }
}
