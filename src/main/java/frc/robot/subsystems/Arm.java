
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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPoints;
import frc.robot.Configs; 

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Arm extends SubsystemBase {

    static SparkMax m_arm = new SparkMax(DriveConstants.armID, MotorType.kBrushless);
    static SparkClosedLoopController m_armController = m_arm.getClosedLoopController();
    private static double armCurrentTarget = ArmPoints.lvl_one;


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

     public static void moveToSetpoint() {
        m_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
      }
    
}
