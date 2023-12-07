package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;



public final class Constants {

    //list with motors and their parameters saved
    public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

    public final static class ControllerConstants{

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVER_FORWARD_DEADBAND = .05;
        public static final double DRIVER_TURN_DEADBAND = .05;

    }
    
    public final static class DrivetrainConstants{

        //Physical Drivetrain Constants *Physical Unit Of Measurement is Meters
        public static final double TRACKWIDTH = 27.88;
        public static final double WHEEL_DIAMETER = 6.0;
        public static final double WHEEL_CIRCUMFERANCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVETRAIN_GEAR_RATIO = 12.75;
        public static final double DRIVING_ENCODER_POS_FACTOR = WHEEL_CIRCUMFERANCE/DRIVETRAIN_GEAR_RATIO;//Unit is Meters
        public static final double DRIVING_ENCODER_VEL_FACTOR = DRIVING_ENCODER_POS_FACTOR/60; //m/s

       public static final double DRIVE_SPEED_MULT = 1;
       public static final double TURN_SPEED_MULT = 1;

        // MOTOR CAN ID Constants
        public static final int LEFT_LEAD_MOTOR_CAN_ID = 3;
        public static final int LEFT_FOLLOWER_MOTOR_CAN_ID = 4;

        public static final int RIGHT_LEAD_MOTOR_CAN_ID = 1;
        public static final int RIGHT_FOLLOWER_MOTOR_CAN_ID = 2;

        public static final boolean RIGHT_MOTORS_INVERT = false;
        public static final boolean LEFT_MOTORS_INVERT = false;

        // Senor CAN ID Constants
        //public static final int PIGEON_GYRO_CAN_ID;
        
        //Math LOL Constants
        public static final double SLEW_RATE_DRIVE_POSITIVE = 8;
        public static final double SLEW_RATE_DRIVE_NEGATIVE = -8;
        public static final double SLEW_RATE_TURN_POSITIVE = 8;
        public static final double SLEW_RATE_TURN_NEGATIVE = -8;

    }
}
