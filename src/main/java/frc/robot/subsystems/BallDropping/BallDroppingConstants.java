// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class BallDroppingConstants {
    public static class AngleMotor{
        //TODO: Find all constants!!!

        public static final double ANGLE_TO_REACH_TOP = 0.4;
        public static final double ANGLE_TO_REACH_BOTTOM = 0.25;
        public static final double ANGLE_TO_RESET = 0;

        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int ID = 24;
        public static final double GEAR_RATIO = 9;
        public static final boolean IS_INVERTED = false;

        public static final double MOTOR_FEED_FORWARD = 0.2;

        public static final double ANGLE_TOLERANCE = 0.03;

        public static final PIDFGains ANGLE_PID = new PIDFGains(1,0,0);
    }


    public static class DropperMotor{
        public static final double POWER_TO_REACH = 0.8; //what power to use for the drop
        public static final boolean IS_INVERTED = true;
        public static final int ID = 20;
        public static final double MAX_DROPPER_POWER = 0.8;
    }

    public static double HIGH_BALL_DROP_TIME = 1;
    public static double LOW_BALL_DROP_TIME = 1;

    
    public static final double X_ON_ROBOT = 0.24;
    public static final double Y_ON_ROBOT = 0.13;
    public static final double Z_OFFSET = 0.87;
}