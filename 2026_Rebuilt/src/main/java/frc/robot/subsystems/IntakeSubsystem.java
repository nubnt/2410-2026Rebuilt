package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private SparkMax intakeRoller = new SparkMax(0, MotorType.kBrushless);


    public void intake(){
        intakeRoller.set(1);
    }

    public void outake(){
        intakeRoller.set(-1);
    }

}
