package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;



public final class Constants {

    //list with motors and their parameters saved
    public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

    public final static class ControllerConstants{
        private static final double DRIVER_FORWARD_DEADBAND = .05;
        private static final double DRIVER_TURN_DEADBAND = .05;

    }
    
    public final static class DrivetrainConstants{

        //Physical Drivetrain Constants *Physical Unit Of Measurement is Meters
        public static final double TRACKWIDTH;
        public static final double WHEEL_DIAMETER;
        public static final double WHEEL_CIRCUMFERANCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVETRAIN_GEAR_RATIO;
        public static final double DRIVING_ENCODER_POS_FACTOR = WHEEL_CIRCUMFERANCE/DRIVETRAIN_GEAR_RATIO;//Unit is Meters
        public static final double DRIVING_ENCODER_VEL_FACTOR = DRIVING_ENCODER_POS_FACTOR/60; //m/s

       // public static final double DRIVING_SPEED_MULTIPLIER = 1;

        // MOTOR CAN ID Constants
        public static final int LEFT_LEAD_MOTOR_CAN_ID = 2;
        public static final int LEFT_FOLLOWER_MOTOR_CAN_ID = 3;

        public static final int RIGHT_LEAD_MOTOR_CAN_ID = 1;
        public static final int RIGHT_FOLLOWER_MOTOR_CAN_ID = 0;

        public static final boolean RIGHT_MOTORS_INVERT = false;
        public static final boolean LEFT_MOTORS_INVERT = false;

        // Senor CAN ID Constants
        public static final int PIGEON_GYRO_CAN_ID;
        
        //Math LOL Constants
        public static final double SLEW_RATE_DRIVE_POSITIVE = 5;
        public static final double SLEW_RATE_DRIVE_NEGATIVE = -5;
        public static final double SLEW_RATE_TURN_POSITIVE = 5;
        public static final double SLEW_RATE_TURN_NEGATIVE = -5;

    }
}
