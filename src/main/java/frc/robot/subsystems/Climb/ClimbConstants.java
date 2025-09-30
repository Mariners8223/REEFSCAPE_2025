// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

/** Add your docs here. */
public class ClimbConstants {
    public static final int MOTOR_ID = 18;
    public static final double GEAR_RATIO = 15;
    public static final double ROTATIONS_TO_METERS = 0.024 * 2 * Math.PI;
    
    public static final double CLIMB_POWER = -1;//-0.65
    public static final double SOFT_MINIMUM = -9.5 * 1; //-9.5 * 0.78


    public static final double SOFT_MAXIMUM = 0;
    public static final boolean IS_INVERTED = true;
    public static final double START_POSITION = 0.125;
    public static final double MOMENT_OF_INERTIA = 1;
}
