package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import java.io.Console;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.SparkBase.ResetMode; 
import frc.robot.Configs; 

public class Intake extends SubsystemBase{
  public static SparkMax m_intake = new SparkMax(DriveConstants.intakeID, MotorType.kBrushed);
  //public static final RelativeEncoder m_intakeencoder = m_intake.getEncoder(); 


  public Intake() {
    
    m_intake.configure(Configs.arm.intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public void setSpeed(double speed) {
    System.out.println("setting intake");

    m_intake.set(speed);
  }

  
    
}
