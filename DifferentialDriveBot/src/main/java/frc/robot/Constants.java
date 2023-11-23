package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {

    public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

    public final static class ControllerConstants{

    }
    
    public final static class DrivetrainConstants{

        //Physical Drivetrain Constants *Physical Unit Of Measurement is Meters
        public static final double WHEEL_DIAMETER;
        public static final double WHEEL_CIRCUMFERANCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVETRAIN_GEAR_RATIO;
        public static final double DRIVING_ENCODER_POS_FACTOR = WHEEL_CIRCUMFERANCE/DRIVETRAIN_GEAR_RATIO;//Unit is Meters
        public static final double Driving_ENCODER_VEL_FACTOR = DRIVING_ENCODER_POS_FACTOR/60; //m/s

       // public static final double DRIVING_SPEED_MULTIPLIER = 1;

        //Motor CAN ID Constants
        public static final int LEFT_LEAD_MOTOR_CAN_ID = 2;
        public static final int LEFT_FOLLOWER_MOTOR_CAN_ID = 3;

        public static final int RIGHT_LEAD_MOTOR_CAN_ID = 1;
        public static final int RIGHT_FOLLOWER_MOTOR_CAN_ID = 0;

        public static final boolean RIGHT_MOTORS_INVERT = false;
        public static final boolean LEFT_MOTORS_INVERT = false;
    }
}
