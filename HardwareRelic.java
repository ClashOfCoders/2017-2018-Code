package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRelic
{
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor wheelOne = null;
    public DcMotor wheelTwo = null;
    public DcMotor wheelThree = null;
    public DcMotor wheelFour = null;

    public DcMotor CubeLift1 = null;
    public DcMotor CubeLift2 = null;
    public CRServo CubeTimeBelt= null;
    public Servo   CubeClawLeft=null;
    public Servo   CubeClawRight=null;
    public Servo   CubeLiftArm=null;

    public TouchSensor CubeSlideSwitchR = null;
    public TouchSensor CubeSlideSwitchL = null;
    public TouchSensor CubeLiftSwitch = null;


    public Servo extender = null;
    public CRServo relicClawLifter = null;


    public Servo RelClaw = null;
    public Servo RelicExtender = null;

    public Servo GemExtender = null;
    public Servo GemRotater =null;

    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public static final int LED_CHANNEL = 5;
    public ModernRoboticsI2cGyro gyro    = null;

    static final double INCREMENT   = 0.05;     // amount to ramp motor each CYCLE_MS cycle
    static final long    CYCLE_MS    =   10;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor
    double  power   = 0;
    boolean rampUp  = true;

    public double GemExtenderInitialPosition = 1;
    public double GemExtenderMiddlePosition = 0.6;
    public double GemExtenderLastPosition = 0.3;
    public double GemRotatorInitialPosition = 0.9;
    public double GemRotatorMidPosition = 0.5;
    public double GemRotatorLastPosition = 0;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRelic(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //  telemetry.addData("Status", "Initialized");


        wheelOne = hwMap.dcMotor.get("wheelOne");
        wheelTwo = hwMap.dcMotor.get("wheelTwo");
        wheelThree = hwMap.dcMotor.get("wheelThree");
        wheelFour = hwMap.dcMotor.get("wheelFour");

        wheelOne.setDirection(DcMotor.Direction.FORWARD);//
        wheelTwo.setDirection(DcMotor.Direction.FORWARD); //
        wheelThree.setDirection(DcMotor.Direction.FORWARD);//
        wheelFour.setDirection(DcMotor.Direction.FORWARD); //

        wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelOne.setPower(0);
        wheelTwo.setPower(0);
        wheelThree.setPower(0);
        wheelFour.setPower(0);


        CubeLift1 = hwMap.get(DcMotor.class,"CubeLift1");
        CubeLift2 = hwMap.get(DcMotor.class,"CubeLift2");
        CubeLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CubeLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CubeTimeBelt= hwMap.crservo.get("CubeTimeBelt");
        CubeTimeBelt.setPower(0);

        CubeClawLeft=hwMap.get(Servo.class, "CubeClawLeft");
        CubeClawRight=hwMap.get(Servo.class,"CubeClawRight");
        CubeClawLeft.setPosition(0.4);
        CubeClawRight.setPosition(0.6);


        CubeLiftArm=hwMap.get(Servo.class,"CubeLiftArm");
        CubeLiftArm.setPosition(0.8);

        CubeSlideSwitchL = hwMap.get(TouchSensor.class,"CubeSlideSwitchL");
        CubeSlideSwitchR = hwMap.get(TouchSensor.class,"CubeSlideSwitchR");
        CubeLiftSwitch = hwMap.get(TouchSensor.class,"CubeLiftSwitch");





        RelClaw= hwMap.get(Servo.class, "RelClaw");
        relicClawLifter = hwMap.crservo.get("relicClawLifter");


        relicClawLifter.setPower(0);
        RelClaw.setPosition(0.0);

        GemExtender= hwMap.get(Servo.class, "GemExtender");
        GemRotater = hwMap.get(Servo.class, "GemRotater");
        cdim = hwMap.deviceInterfaceModule.get("cdim");
        sensorRGB = hwMap.colorSensor.get("sensorRGB");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        RelicExtender = hwMap.get(Servo.class, "RelicExtender");
        RelicExtender.setPosition(0.52);

        boolean bLedOn = false;
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        GemRotater.setPosition(GemRotatorInitialPosition);
        GemExtender.setPosition(GemExtenderInitialPosition);




        //stopRobot();

    }



    public void setDriveEncoderMode( Boolean setOrStop){
        if(setOrStop) {
            wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            wheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelFour.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    public void setDriveNonEncoderMode( ){

        wheelOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stopRobot () {
        wheelOne.setPower(0);
        wheelTwo.setPower(0);
        wheelThree.setPower(0);
        wheelFour.setPower(0);
        CubeLift1.setPower(0);
        CubeLift2.setPower(0);
        CubeTimeBelt.setPower(0);
        relicClawLifter.setPower(0);

    }



    public void setDriveSpeed(double leftSpeed,double rightSpeed, int driveDirection){
        wheelOne.setPower(leftSpeed*driveDirection);
        wheelTwo.setPower(leftSpeed*driveDirection);
        wheelThree.setPower(rightSpeed*driveDirection);
        wheelFour.setPower(rightSpeed*driveDirection);

    }

    public double gyroAngle(double gyroReading){
        double newGyro=gyroReading % 360;
        if(newGyro>180) newGyro=newGyro-360;
        return  newGyro;
    }

    public double getAngle(double x, double y){
        double hypotneus;
        double angle = 0;

        hypotneus = Math.sqrt(x*x + y*y);

        y = -y;

        if (x>0 && y>0){
            angle = Math.toDegrees(Math.asin(x/hypotneus));
        } else if (x>0 && y<0){
            angle = Math.toDegrees(Math.asin(Math.abs(y)/hypotneus)) + 90;
        } else if (x < 0 && y < 0){
            angle = Math.toDegrees(Math.asin(Math.abs(x)/hypotneus)) + 180;
        } else if (x < 0 && y > 0){
            angle = Math.toDegrees(Math.asin(y/hypotneus)) + 270;
        } else if (x == 0) {
            if (y > 0)
                angle = 0;
            else if (y < 0)
                angle = 180;
        }else if (y == 0) {
            if (x > 0)
                angle = 90;
            else if (x < 0)
                angle = 270;
        }else
            angle = -1;

        return angle;
    }

    public double[] calculateDrivingSpeed(double forward, double horizontal, double rotation) {

        double[] speedArray = new double[4];

        speedArray[0] = forward + horizontal + rotation;
        speedArray[1] = -forward + horizontal + rotation;
        speedArray[2] = -forward - horizontal + rotation;
        speedArray[3] = forward - horizontal + rotation;

        double maxValue=0;
        int i;

       /* for (i=0;i<=3;i++) {
            maxValue = Math.max(speedArray[i], maxValue);
        }
        if (Math.abs(maxValue)>1){
        if (maxValue!=0) {
            for (i = 0; i <= 3; i++) {
                speedArray[i] = speedArray[i] / Math.abs(maxValue);
            }
        }

        }
        */

        return speedArray;
    }

    public double gyroCorrection(double rotation){
        double gyroRotation = rotation+ 0.1*(rotation-0.001*gyro.getIntegratedZValue());
        gyroRotation=Range.clip(gyroRotation,-1,1);
        return gyroRotation;
    }


    // This program simply drives the robot forward with an specified angle
    public void angleDrive (OpMode currentOpMode,double speed, double angle, double rotation){

        double v1, v2;
/*
        if (Math.abs(angle) % 90 < Math.abs((angle % 90) - 45)){
            v1 = speed;
            v2 = -speed;
        } else {
            v1 = speed;
            v2 = 0;
        }
*/

        v1 = speed;
        v2 = -speed;

       // v1/=2;
        //v2/=2;

        //the motors are configured to go counterclockwise if the power are all set to 1
        v1 = -v1;
        v2 = -v2;

        rotation/=-2;

        if (angle >= 0 && angle <90)
            setSpeed(currentOpMode,v1 + rotation, v2 + rotation, -v1 + rotation, -v2 + rotation);
        else if (angle >= 90 && angle < 180)
            setSpeed(currentOpMode,-v2 + rotation, v1 + rotation, v2 + rotation, -v1 + rotation);
        else if (angle >= 180 && angle < 270)
            setSpeed(currentOpMode,-v1 + rotation, -v2 + rotation, v1 + rotation, v2 + rotation);
        else if (angle >= 270 && angle < 360)
            setSpeed(currentOpMode,v2 + rotation, -v1 + rotation, -v2 + rotation, v1 + rotation);
        else
            setSpeed(currentOpMode,0, 0, 0, 0);

    }





    public void setSpeed(OpMode currentOpMode, double one, double two, double three, double four) {


        double rampStep1=one-wheelOne.getPower();
        double rampStep2=two-wheelTwo.getPower();
        double rampStep3=three-wheelThree.getPower();
        double rampStep4=four-wheelFour.getPower();

        double maxRamp=Math.max(Math.abs(rampStep1),Math.abs(rampStep2));
        double maxRamp2=Math.max(maxRamp,Math.abs(rampStep3));
        double maxRamp3=Math.max(maxRamp2,Math.abs(rampStep4));

        int rampsteps= (int) (maxRamp3/INCREMENT);
        if (rampsteps<1)
                rampsteps=1;


        int iteration=0;
        double currentPower1=wheelOne.getPower();
        double currentPower2=wheelTwo.getPower();
        double currentPower3=wheelThree.getPower();
        double currentPower4=wheelFour.getPower();




        while(currentOpMode.getRuntime()<150 && iteration<rampsteps) {

           currentPower1 += rampStep1/rampsteps;
           currentPower2 += rampStep2/rampsteps;
           currentPower3 += rampStep3/rampsteps;
           currentPower4 += rampStep4/rampsteps;


            // Set the motor to the new power and pause;
            wheelOne.setPower(currentPower1);
            wheelTwo.setPower(currentPower2);
            wheelThree.setPower(currentPower3);
            wheelFour.setPower(currentPower4);
            try {
                Thread.sleep(CYCLE_MS);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            iteration +=1;

        }
        if (currentOpMode.getRuntime()>150)
            stopRobot();


    }


    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}