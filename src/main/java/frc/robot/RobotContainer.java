package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PathfindingCommand;

import java.util.Set;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.Positions;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import java.util.Map;

import static frc.robot.Constants.Elevator.LEVEL_TWO;
import static frc.robot.Robot.AutoModeChooser;
import static frc.robot.Robot.ControlModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final ElevatorSubsytem Elevator =new ElevatorSubsytem();
    public final ArmSubsytem Arm= new ArmSubsytem();
    public final RampSubSystem Ramp = new RampSubSystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);

    //private final TimedSpeakerShotCommand TimedSpeakerShot = new TimedSpeakerShotCommand(Shooter);

    private final GenericEntry finalSpeedModifierEntry = Shuffleboard.getTab("config").add("final speed modifier", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.OPERATOR_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ControlModeChooser.onChange((ControlMode mode) -> {
            if (mode == ControlMode.SINGLE) {
                OPERATOR = new Gamepad(Controller.DRIVER_PORT);
            } else {
                OPERATOR = new Gamepad(Controller.OPERATOR_PORT);
            }
            configureDefaultCommands();
            configureButtonBindings();
        });

        Shuffleboard.getTab("main").add("swerve", Swerve);
       // Shuffleboard.getTab("main").add("shooter", Shooter);
/* 
        AutoBuilder.configureHolonomic(
                PoseEstimation::getCurrentPose,
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,
                Constants.Auto.PATH_FOLLOWER_CONFIG,
                () -> Robot.alliance == DriverStation.Alliance.Red,
                Swerve
        );*/

        AutoBuilder.configure(
                PoseEstimation::getCurrentPose, // Robot pose supplier
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
           Constants.Auto.PATH_FOLLOWER_CONFIG, // The path follower configuration
            Constants.Auto.config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            Swerve // Reference to this subsystem to set requirements
    );
        PathfindingCommand.warmupCommand().schedule();
  


        NamedCommands.registerCommand("L4_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.LEVELFOUR, Ramp), Elevator));
        NamedCommands.registerCommand("L1_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.LEVELONE, Ramp), Elevator));
        NamedCommands.registerCommand("Intake_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.INTAKE, Ramp), Elevator));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));
       // NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> Shooter.receive(false), Shooter));
       // NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> Shooter.flywheelSpeaker(true), Shooter));
       // NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> Shooter.flywheelSpeaker(false), Shooter));
        //NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> Shooter.intake(true, false), Shooter));
        //NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> Shooter.intake(false, false), Shooter));
        //NamedCommands.registerCommand("lowerIntake", new InstantCommand(() -> LinearActuator.shift(false, true), LinearActuator));
        //NamedCommands.registerCommand("lowerIntakeStop", new InstantCommand(() -> LinearActuator.shift(false, false), LinearActuator));

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        Swerve.setDefaultCommand(
                new TeleopSwerveCommand(
                        Swerve,
                        () -> -1 * DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getLeftX(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false,
                        () -> false,// DRIVER.getLeftStickButton(), // slow mode
                        () -> false,//DRIVER.getRightStickButton())); // fast mode
                        () -> finalSpeedModifierEntry.getDouble(1.0)));
               
         Elevator.setDefaultCommand(
                new RunCommand(
                        () -> Elevator.setAngle(false),
                        Elevator));    
        LED.setDefaultCommand(
                new RunCommand(
                        () -> LED.printDetails(),
                        LED));    
       
   }

    private void configureButtonBindings() {
       
                new JoystickButton(DRIVER, 3)
        .onTrue(new RunCommand(() -> Swerve.zeroGyro(),Swerve));

    
               new JoystickButton(OPERATOR, 7)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(true),Elevator));
        
        new JoystickButton(OPERATOR, 8)
        .whileTrue(new RunCommand(() -> Elevator.setNAngle(true),Elevator));
      
        new JoystickButton(OPERATOR, 3)
        .onTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELONE,Ramp),Elevator));
        new JoystickButton(OPERATOR, 4)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELTWO,Ramp),Elevator));
        new JoystickButton(OPERATOR, 6)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELTHREE,Ramp),Elevator));
        new JoystickButton(OPERATOR, 5)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELFOUR,Ramp),Elevator));
        new JoystickButton(OPERATOR, 1)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(Positions.INTAKE,Ramp),Elevator));
        new JoystickButton(OPERATOR, 2)
        .whileTrue(new RunCommand(() -> Ramp.setPosScore(true),Ramp));

        new JoystickButton(DRIVER, 1).
        whileTrue(Swerve.driveToPose());
     
       }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(AutoModeChooser.getSelected().pathplannerName);
    }
    public void checkAnalogs(){
        if (OPERATOR.getRightTriggerAxis()>.5){
                        CommandScheduler.getInstance().schedule(new RunCommand(() -> Ramp.setPosReceive(true),Ramp));
                        CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));


        }
        if (OPERATOR.getLeftTriggerAxis()>.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Ramp.setPosCarrying(true),Ramp));
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));


}
        if (OPERATOR.getRightY()>.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Arm.ShootArm(true),Arm));

        }
        else{
                 CommandScheduler.getInstance().schedule(new RunCommand(() -> Arm.ShootArm(false),Arm));           
        }
        if (OPERATOR.getRightY()<-.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Arm.NegativeShootArm(true),Arm));
        }
        


    }
}
