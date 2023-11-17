package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;





public final class Constants {

    public static ArrayList<CANSparkMax> SPARK_LIST = new ArrayList<CANSparkMax>();

    public final static class ControllerConstants{

    }
    
    public final static class DrivetrainConstants{


        //Motor CAN ID Constants
        public static final int LEFT_LEAD_MOTOR_CAN_ID = 2;
        public static final int LEFT_FOLLOWER_MOTOR_CAN_ID = 3;

        public static final int RIGHT_LEAD_MOTOR_CAN_ID = 1;
        public static final int RIGHT_FOLLOWER_MOTOR_CAN_ID = 0;

        public static final boolean RIGHT_MOTORS_INVERT = false;
        public static final boolean LEFT_MOTORS_INVERT = false;
    }
}
