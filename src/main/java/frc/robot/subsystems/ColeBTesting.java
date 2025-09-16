package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Configs;

public class ColeBTesting{

    private double motorSpeed = 0.25;

    private SparkMax coleMotor =
        new SparkMax(1, MotorType.kBrushless);
        
    
}