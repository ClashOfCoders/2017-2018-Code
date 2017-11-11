package org.firstinspires.ftc.teamcode;

/**
 * Created by jianqiuzhang on 11/10/17.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

public class HardWare2018 {
    public DcMotor  WheelFrontLeft   = null;
    public DcMotor  WheelFrontRight  = null;
    public DcMotor  WheelBackLeft     = null;
    public DcMotor  WheelBackRight     = null;
   /* public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    */

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardWare2018(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        WheelFrontLeft  = hwMap.get(DcMotor.class, "WheelFrontLeft");
        WheelFrontRight = hwMap.get(DcMotor.class, "WheelFrontRight");
        WheelBackLeft  = hwMap.get(DcMotor.class, "WheelBackLeft");
        WheelBackRight = hwMap.get(DcMotor.class, "WheelBackRight");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        WheelFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        WheelFrontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        WheelBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        WheelBackRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);
        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        /*
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        */
    }
    public void stopRobot(){
        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);

    }
}
