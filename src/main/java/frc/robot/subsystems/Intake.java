
package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPoints;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPoints;
import frc.robot.Configs; 

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Intake extends SubsystemBase {

    public static SparkMax m_intake = new SparkMax(DriveConstants.intakeID, MotorType.kBrushed);

    public Intake(){
      
        m_intake.configure(Configs.arm.arm_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


      public static void setIntakeSpeed(double speed) {

        m_intake.set(speed);
      }  
    
}
