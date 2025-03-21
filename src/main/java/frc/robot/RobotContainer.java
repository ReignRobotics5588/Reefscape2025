// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Configs.arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Climber; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.servoMotor;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final Climber m_robotClimb = new Climber();
  public static final Arm m_robotArm = new Arm();
  public static final Elevator m_robotElevator = new Elevator();
  public static final Intake m_robotintake = new Intake(); 
  public static final servoMotor m_servo = new servoMotor();


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  XboxController m_climberController = new XboxController(2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true); // change for comp

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*.9, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*.9, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*.9, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

        m_robotClimb.setDefaultCommand(
        new RunCommand(
          ()-> m_robotClimb.setSpeed(
              -MathUtil.applyDeadband(m_climberController.getLeftY(), OIConstants.kDriveDeadband))
          , m_robotClimb));

        // subsystems
        m_robotArm.setDefaultCommand(
        new RunCommand(
            ()-> m_robotArm.setArmSpeed(
                -MathUtil.applyDeadband(m_operatorController.getLeftY()*-0.25, OIConstants.kDriveDeadband))
            , m_robotArm));

        m_robotElevator.setDefaultCommand(
        new RunCommand(
            ()-> m_robotElevator.setSpeed(
                -MathUtil.applyDeadband(m_operatorController.getRightY()*.40, OIConstants.kDriveDeadband))
            , m_robotElevator));


            m_robotintake.setDefaultCommand(
              new RunCommand(
                  ()-> m_robotintake.setSpeed(0)
                  , m_robotintake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    
      // button y
      
      new JoystickButton(m_climberController, Button.kTriangle.value)
      .onTrue (new RunCommand(
            () -> m_servo.setServoPosition(1),
            m_servo));

            // button A
            new JoystickButton(m_climberController, Button.kSquare.value)
            .onTrue (new RunCommand(
                  () -> m_servo.setServoPosition(0.2),
                  m_servo));

                  new JoystickButton(m_climberController, Button.kCircle.value)
                  .onTrue (new RunCommand(
                        () -> m_servo.setServoPosition(0),
                        m_servo));
      
    /** 

     new JoystickButton(m_climberController, Button.kTriangle.value)
        .onTrue (new RunCommand(
              () -> m_robotElevator.setPosition(0.42),
                  m_robotElevator));

     new JoystickButton(m_climberController, Button.kTriangle.value)
         .onFalse (new RunCommand(
             () -> m_robotElevator.setSpeed(0),
                  m_robotElevator));
    */
        
      new JoystickButton(m_operatorController, Button.kR1.value)
      .onFalse (new RunCommand(
            () -> m_robotintake.setSpeed(ArmConstants.kIntakeStopSpeed),
            m_robotintake));

    new JoystickButton(m_operatorController, Button.kR1.value)
      .onTrue (new RunCommand(
            () -> m_robotintake.setSpeed(ArmConstants.kIntakeUpSpeed),
            m_robotintake));

            new JoystickButton(m_operatorController, Button.kL1.value)
            .onTrue (new RunCommand(
            () -> m_robotintake.setSpeed(ArmConstants.kIntakeDownSpeed),
            m_robotintake));
      
          new JoystickButton(m_operatorController, Button.kL1.value)
            .onFalse (new RunCommand(
            () -> m_robotintake.setSpeed(ArmConstants.kIntakeStopSpeed),
            m_robotintake));
      



  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("AUTO");
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
        //
    

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    

    return Commands.waitSeconds(2).deadlineFor(new RunCommand(
      () -> {
        // System.out.println("RUNNING");
        m_robotDrive.drive(
          -0.6,
          0.0,
          0.0,
          false);
      },
      m_robotDrive)
    );
    

    // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    // return Commands.runOnce(()-> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()));
  }

  public double getArmVelocity(){
    return m_robotArm.getVelocity();
  }
  public double getElevatorVelocity(){
    return m_robotElevator.getVelocity();
  }
  public double getClimberVelocity(){
    return m_robotClimb.getVelocity();
  }


  public double getArmPosition(){
    return m_robotArm.getPosition();
  }
  public double getElevatorPosition(){
    return m_robotElevator.getPosition();
  }
  public double getClimberPosition(){
    return m_robotClimb.getPosition();
  }
}
