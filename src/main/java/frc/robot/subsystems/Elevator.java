
package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode; 
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/*
 * COPIED FROM ARM BRANCH
 * 
 * package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode; 

/*
* limit switch, to know when we reached a certain point
* mechanism pulls in and stops the winch
* get a true false reading, signal pin should always be 5 or 0 bolts
* idfk read limit switch documentation
* will forward be extending or pulling?????????????
* run motor in other direction, not same direction anymore
* SparkMax motors connecting directly to the motor controller?????????????
* NEO motors 
* DIO pins
* use NO and CLOSE
* 10 and 11
*/
/* 
public class Climber extends SubsystemBase {

    public SparkMax m_climber = new SparkMax(ClimberConstants.kClimberLeft, MotorType.kBrushless);
    public SparkMaxConfig climber_config; 
  
    public DigitalInput Switch = new DigitalInput(5);
    public RelativeEncoder m_encoder = m_climber.getEncoder();
  
    public Climber() {
      /** 
      m_climber.setIdleMode(IdleMode.kBrake);
      m_encoder.setPosition(0.0);
  
      m_climber.setSmartCurrentLimit(30);
      
      m_climber.setIdleMode(IdleMode.kBrake); //keeps climber in break mode from the
      // code itself :)
      */
  
      /*climber_config = new SparkMaxConfig();
  
      climber_config
          .inverted(true)
          .idleMode(IdleMode.kBrake);
      climber_config.encoder
          .positionConversionFactor(1000)
          .velocityConversionFactor(1000);
      climber_config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(1.0, 0.0, 0.0);
      
      m_climber.configure(climber_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    public void setSpeed(double speedLeft, double speedRight) {
  
      if ((Math.abs(speedLeft) > 0.25) || (Math.abs(speedRight))  > 0.25) {
          m_climber.set(speedLeft*ClimberConstants.kClimberSpeedLimit);
      }
    }
  
    public boolean getSwitch() {
      return Switch.get();
    }
  
    public double getLeftEncoderDistance() {
      return (m_climber.getEncoder().getPosition() / 42);// 42 ticks in one rotation, counting how many full rotations there are :)
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  }

  */

public class Elevator {

    SparkMax m_elevator = new SparkMax(DriveConstants.elevatorCANId, MotorType.kBrushless);
    public SparkMaxConfig elevator_config;

    public Elevator(){

        m_elevator.setIdleMode(IdleMode.kBrake);
        elevator_config = new SparkMaxConfig(); 

        elevator_config
          .inverted(true)
          .idleMode(IdleMode.kBrake);
        elevator_config.encoder
          .positionConversionFactor(1000)
          .velocityConversionFactor(1000);
        elevator_config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(1.0, 0.0, 0.0);
      
        elevator_config.configure(elevator_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    public static void setSpeed(double speed) {
  
        if ((Math.abs(speed) > 0.25)) {
            m_elevator.set(speed);
        }
      }
    
}
