package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class StupidAlignCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final LEDSubsystem LED;
    private final boolean isLeft;

    private IntegerSubscriber idSub;
    private DoubleSubscriber txSub;
//    private DoubleSubscriber tySub;
    private DoubleSubscriber tzSub;
    private DoubleSubscriber yawSub;
    private DoubleSubscriber pitchSub;
    private DoubleSubscriber rollSub;

    private final SwerveDrivePoseEstimator poser = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_KINEMATICS,
            new Rotation2d(),
            new SwerveModulePosition[]{
                    new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(1.5, 1.5, 1.5),//VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(1.5, 1.5, 1.5)
    );


    public StupidAlignCommand(
            boolean isLeft,
            SwerveSubsystem Swerve,
            LEDSubsystem LED
    ) {
        this.isLeft = isLeft;
        this.Swerve = Swerve;
        this.LED = LED;

        var ntInstance = NetworkTableInstance.getDefault();
        var ntTable = ntInstance.getTable("datatable");
        idSub = ntTable.getIntegerTopic("id").subscribe(0);
        txSub = ntTable.getDoubleTopic("tx").subscribe(0);
//        tySub = ntTable.getDoubleTopic("ty").subscribe(0);
        tzSub = ntTable.getDoubleTopic("tz").subscribe(0);
        yawSub = ntTable.getDoubleTopic("yaw").subscribe(0);
        pitchSub = ntTable.getDoubleTopic("pitch").subscribe(0);
        rollSub = ntTable.getDoubleTopic("roll").subscribe(0);

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        if (idSub.get() == 0) {
            System.out.println("align: no tag");
            LED.setPattern(LEDSubsystem.BlinkinPattern.DARK_RED);
            this.cancel();
        } else {
            System.out.println("align: found tag, aligning");
            LED.setPattern(LEDSubsystem.BlinkinPattern.SHOT_BLUE);
        }
    }

    @Override
    public void execute() {
        long id = idSub.get();

        if (id != 0) {
            // tag spotted
            poser.resetPose(new Pose2d(txSub.get(), tzSub.get(), new Rotation2d()));
        }
            // use wheel odom instead
        var positions = Swerve.getPositions();
//        for (int i = 0; i < positions.length; i++) {
//            positions[i].distanceMeters *= -1;
//        }
        poser.update(Swerve.getYaw(), positions);

        System.out.println(poser.getEstimatedPosition().toString());

        //System.out.println("id: " + id);
        double diffX = poser.getEstimatedPosition().getX() - (isLeft ? 1 : -1) * 0.172;//xGoal.getDouble(0.17);
        double diffY = poser.getEstimatedPosition().getY() - .65;//yGoal.getDouble(0.7);
       // System.out.println("tz: "+ tzSub.get() + "ygoal: "+yGoal.getDouble(.7));
//        double diffTheta = yawSub.get() - thetaGoal.getDouble(0);
//        System.out.println("diffX=" + diffX + " diffY=" + diffY + " diffTheta=" + diffTheta);

        double speed = .3;//speedEntry.getDouble(0);
        double goX = diffX > 0 ? speed : -speed;
        double goY = diffY > 0 ? speed : -speed;
//        double goTheta = diffTheta > 0 ? -speed : speed;

        if (Math.abs(diffX) < 0.01) {
            goX = 0;
        }
        if (Math.abs(diffY) < 0.01) {
            goY = 0;   
        }

        if (goX == 0 && goY == 0) {
            System.out.println("align: close enough");
            LED.setPattern(LEDSubsystem.BlinkinPattern.LIME);
            this.cancel();
        } else {
            Swerve.drive(new Translation2d(goX, goY), /*goTheta*/0, false, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        poser.resetPosition(new Rotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, new Pose2d());
    }
}