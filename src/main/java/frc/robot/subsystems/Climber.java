package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.spark.SparkBase.ResetMode; 
import frc.robot.Configs; 

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
*10 and 11
*/

public class Climber extends SubsystemBase {

  public static SparkMax m_climber = new SparkMax(ClimberConstants.kClimber, MotorType.kBrushless);
  public static final RelativeEncoder m_climberencoder = m_climber.getEncoder(); 
  public RelativeEncoder m_encoder = m_climber.getEncoder();
   public static final Servo m_servo = new Servo(1);

  public Climber() {
    /** 
    m_climber.setIdleMode(IdleMode.kBrake);
    m_encoder.setPosition(0.0);

    m_climber.setSmartCurrentLimit(30);
    
    m_climber.setIdleMode(IdleMode.kBrake); //keeps climber in break mode from the
    // code itself :)
    */
    
    m_climber.configure(Configs.Climber.climber_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {

    if (Math.abs(speed) < 0) {
        speed *= ClimberConstants.kClimberUpperSpeed;
    } else {
        speed *= ClimberConstants.kClimberDownSpeed;
    }
    m_climber.set(speed);
  }

  public double getLeftEncoderDistance() {
    return (m_climber.getEncoder().getPosition() / 42);// 42 ticks in one rotation, counting how many full rotations there are :)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public double getVelocity(){
    return m_climberencoder.getVelocity();
  }

  public double getPosition(){
    return m_climberencoder.getPosition();
  }

  public static void setServoPosition(double pos){
    m_servo.set(pos); // (left) 0.0 to 1.0 (all the way over?)
  }

  public static void setServoAngle(double pos){
    m_servo.setAngle(pos);
  }
  
  
}