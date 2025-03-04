package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Elevator.Positions;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import java.util.Map;

import static frc.robot.Robot.AutoModeChooser;
import static frc.robot.Robot.ControlModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final ElevatorSubsystem Elevator =new ElevatorSubsystem();
    public final ClimberSubsystem Climber = new ClimberSubsystem();
    public final TongueSubsystem Tongue = new TongueSubsystem();
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
            () ->  { 
            // () -> return Robot.alliance == DriverStation.Alliance.Red
            
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            Swerve // Reference to this subsystem to set requirements
    );
        PathfindingCommand.warmupCommand().schedule();
  


        NamedCommands.registerCommand("L4_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.LEVELFOUR, Tongue), Elevator));
        NamedCommands.registerCommand("L1_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.LEVELONE, Tongue), Elevator));
        NamedCommands.registerCommand("Intake_Elevator", new InstantCommand(() -> Elevator.setAngle(Positions.INTAKE, Tongue), Elevator));
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
                        () -> 1 * DRIVER.getLeftX(),
                        () -> -1 * DRIVER.getLeftY(),
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
        .onTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELONE, Tongue),Elevator));
        new JoystickButton(OPERATOR, 4)
        .onTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELTWO, Tongue),Elevator));
        new JoystickButton(OPERATOR, 6)
        .onTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELTHREE, Tongue),Elevator));
        new JoystickButton(OPERATOR, 5)
        .onTrue(new RunCommand(() -> Elevator.setAngle(Positions.LEVELFOUR, Tongue),Elevator));
        new JoystickButton(OPERATOR, 1)
        .whileTrue(new RunCommand(() -> Elevator.setAngle(Positions.INTAKE, Tongue),Elevator));
        new JoystickButton(OPERATOR, 2)
        .whileTrue(new RunCommand(() -> Tongue.setPosScore(true), Tongue));

        new JoystickButton(DRIVER, 1).
        whileTrue(Swerve.driveToPose());
     
       }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(AutoModeChooser.getSelected().pathplannerName);
    }
    public void checkAnalogs(){
        if (OPERATOR.getRightTriggerAxis()>.5){
                        CommandScheduler.getInstance().schedule(new RunCommand(() -> Tongue.setPosReceive(true), Tongue));
                        CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));


        }
        if (OPERATOR.getLeftTriggerAxis()>.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Tongue.setPosCarrying(true), Tongue));
                CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));


}
        if (OPERATOR.getRightY()>.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(true), Climber));

        }
        else{
                 CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(false), Climber));
        }
        if (OPERATOR.getRightY()<-.5){
                CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.NegativeShootArm(true), Climber));
        }
        


    }
}
