
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Configs; 

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Arm {

    static SparkMax m_arm = new SparkMax(DriveConstants.elevatorCANId, MotorType.kBrushless);

    public Arm(){
      
        m_arm.configure(Configs.arm.arm_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    public static void setSpeed(double speed) {
  
        if (Math.abs(speed) < 0) {
            speed *= ArmConstants.kArmUpperSpeed;
          } 
          else {
            speed *= ArmConstants.kArmDownSpeed;
          }

        m_arm.set(speed);
      }
    
}
