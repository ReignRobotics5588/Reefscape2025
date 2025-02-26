
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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import frc.robot.Configs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Elevator extends SubsystemBase {

    static SparkMax m_elevator = new SparkMax(DriveConstants.elevatorCANId, MotorType.kBrushless);
    public static final AbsoluteEncoder m_elevatorencoder = m_elevator.getAbsoluteEncoder(); 
    static SparkClosedLoopController m_elevatorController = m_elevator.getClosedLoopController();
    private static double elevatorCurrentTarget = ElevatorPoints.lvl_one;

    public Elevator(){
      
        m_elevator.configure(Configs.elevator.elevator_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    public static void setSpeed(double speed) {
      m_elevator.set(speed);
    }

    public static void moveToSetpoint() {
      m_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

    public double getVelocity(){
      return m_elevatorencoder.getVelocity();
    }

    public double getPosition(){
      return m_elevatorencoder.getPosition();
    }
    
}
