package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class StupidAlignCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final boolean isLeft;
//    private double initialTimestamp;

    private final GenericEntry secondsToShootEntry = Shuffleboard.getTab("config").add("stupid seconds to shoot", 2.0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 5)).getEntry();


    private final GenericEntry xGoal = Shuffleboard.getTab("config").add("stupid x goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private final GenericEntry yGoal = Shuffleboard.getTab("config").add("stupid y goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private final GenericEntry thetaGoal = Shuffleboard.getTab("config").add("stupid theta goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private boolean isAligned = false;

    private IntegerSubscriber idSub;
    private DoubleSubscriber txSub;
//    private DoubleSubscriber tySub;
    private DoubleSubscriber tzSub;
//    private DoubleSubscriber yawSub;
//    private DoubleSubscriber pitchSub;
//    private DoubleSubscriber rollSub;


    public StupidAlignCommand(
            boolean isLeft,
            SwerveSubsystem Swerve
    ) {
        this.isLeft = isLeft;
        this.Swerve = Swerve;

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
//        initialTimestamp = getFPGATimestamp();

        var ntInstance = NetworkTableInstance.getDefault();
        var ntTable = ntInstance.getTable("datatable");
        idSub = ntTable.getIntegerTopic("id").subscribe(0);
        txSub = ntTable.getDoubleTopic("tx").subscribe(0);
//        tySub = ntTable.getDoubleTopic("ty").subscribe(0);
        tzSub = ntTable.getDoubleTopic("tz").subscribe(0);
//        yawSub = ntTable.getDoubleTopic("yaw").subscribe(0);
//        pitchSub = ntTable.getDoubleTopic("pitch").subscribe(0);
//        rollSub = ntTable.getDoubleTopic("roll").subscribe(0);

        Shuffleboard.getTab("debug").addBoolean("ALIGNED?", () -> isAligned).withWidget(BuiltInWidgets.kBooleanBox);
    }

    @Override
    public void execute() {
        long id = idSub.get();

        //System.out.println("id: " + id);
        double diffX = txSub.get() - (isLeft ? 1 : -1) * 0.17;//xGoal.getDouble(0.17);
        double diffY = tzSub.get() - .7;//yGoal.getDouble(0.7);
       // System.out.println("tz: "+ tzSub.get() + "ygoal: "+yGoal.getDouble(.7));
//        double diffTheta = yawSub.get() - thetaGoal.getDouble(0);
//        System.out.println("diffX=" + diffX + " diffY=" + diffY + " diffTheta=" + diffTheta);

        double speed = .3;//speedEntry.getDouble(0);
        double goX = diffX > 0 ? speed : -speed;
        double goY = diffY > 0 ? speed : -speed;
//        double goTheta = diffTheta > 0 ? -speed : speed;
        if (id == 0) {
            System.out.println("no data");
            goX = -goX; // go back in opposite x direction?
        }

        if (Math.abs(diffX) < 0.05) {
            goX = 0;
        }
        if (Math.abs(diffY) < 0.05) {
            goY = 0;   
        }

        if (goX == 0 && goY == 0) {
            System.out.println("close enough");
            isAligned = true;
        } else {
            Swerve.drive(new Translation2d(goX, goY), /*goTheta*/0, false, true);
            isAligned = false;
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned;
    }

    @Override
    public void end(boolean interrupted) {
    }
}