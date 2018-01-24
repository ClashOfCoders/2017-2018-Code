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

import static java.lang.Runtime.getRuntime;

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
    public Servo   CubeSecondClaw=null;

    public TouchSensor CubeSlideSwitchR = null;
    public TouchSensor CubeSlideSwitchL = null;
    public TouchSensor CubeLiftSwitch = null;
    ModernRoboticsI2cRangeSensor rangeSensor;


    // public CRServo relicClawLifter = null;
    public DcMotor RelicLifter=null;
    public Servo RelClaw = null;
    public CRServo RelicExtender = null;
    public TouchSensor RelicSwitch = null;

    public CRServo GemExtender = null;
    public Servo GemRotater =null;

    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public static final int LED_CHANNEL = 5;
    public ModernRoboticsI2cGyro gyro    = null;

    static final double INCREMENT   = 0.05;     // amount to ramp motor each CYCLE_MS cycle
    static final long    CYCLE_MS    =   10;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    public static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415*Math.sqrt(2)/2);


    public double GemExtenderInitialPosition = -0.1;
    public double GemExtenderMiddlePosition = 0.45;
    public double GemExtenderLastPosition = 0.7;
    public double GemRotatorInitialPosition = 1;
    public double GemRotatorMidPosition = 0.5;
    public double GemRotatorLastPosition = 0;

    public double CubeLiftArmDownPosition=0.1;
    public double CubeLiftArmUpPosition=1;

    public double cubeSecondOpenPos=0.5;
    public double cubeSecondClosePos=0.4;

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
        CubeClawLeft.setPosition(0);
        CubeClawRight.setPosition(1);


        CubeLiftArm=hwMap.get(Servo.class,"CubeLiftArm");
        CubeLiftArm.setPosition(CubeLiftArmDownPosition);


        CubeSlideSwitchL = hwMap.get(TouchSensor.class,"CubeSlideSwitchL");
        CubeSlideSwitchR = hwMap.get(TouchSensor.class,"CubeSlideSwitchR");
        CubeLiftSwitch = hwMap.get(TouchSensor.class,"CubeLiftSwitch");
        CubeSecondClaw = hwMap.get(Servo.class, "CubeSecondClaw");

        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");



        CubeSecondClaw.setPosition(cubeSecondClosePos);





        RelClaw= hwMap.get(Servo.class, "RelClaw");
        RelicExtender = hwMap.crservo.get("RelicExtender");
        //RelicExtender = hwMap.get(Servo.class, "RelicExtender");
        //relicClawLifter = hwMap.crservo.get("relicClawLifter");
        RelicLifter=hwMap.get(DcMotor.class,"RelicLifter");
        RelicSwitch = hwMap.get(TouchSensor.class,"RelicSwitch");

        RelicLifter.setDirection(DcMotor.Direction.FORWARD);//
        RelicLifter.setPower(0);
        RelicLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //relicClawLifter.setPower(0.75);
        RelClaw.setPosition(0.0);
        RelicExtender.setPower(0);
        RelicExtender.setDirection(DcMotorSimple.Direction.FORWARD);

        GemExtender= hwMap.get(CRServo.class, "GemExtender");
        GemRotater = hwMap.get(Servo.class, "GemRotater");
        cdim = hwMap.deviceInterfaceModule.get("cdim");
        sensorRGB = hwMap.colorSensor.get("sensorRGB");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");








        GemRotater.setPosition(GemRotatorInitialPosition);
        GemExtender.setPower(GemExtenderInitialPosition);

        boolean bLedOn = true;
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);
        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

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

        RelicExtender.setPower(0);
        //       CubeLiftArm.setPosition(CubeLiftArmDownPosition);


        //       RelClaw.setPosition(0.0);
        GemRotater.setPosition(GemRotatorInitialPosition);
        GemExtender.setPower(GemExtenderInitialPosition);



    }



    public void setDriveSpeed(double leftSpeed,double rightSpeed, int driveDirection){
        wheelOne.setPower(leftSpeed*driveDirection);
        wheelTwo.setPower(leftSpeed*driveDirection);
        wheelThree.setPower(rightSpeed*driveDirection);
        wheelFour.setPower(rightSpeed*driveDirection);

    }

    public double gyroAngle(double gyroReading){
        double newGyro=gyroReading % 360;
        // if(newGyro>180) newGyro=newGyro-360;
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




        while(currentOpMode.getRuntime()<150 && iteration<rampsteps ) {

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

    public void gyroDrive(LinearOpMode currentOpMode, double speed, double angle, double distance,double timeLimit,double gyroTarget){

        double targetTicks = (distance) * 1120 / (4 * 3.14);


        double pos1 = wheelOne.getCurrentPosition();
        double pos2 = wheelTwo.getCurrentPosition();
        double pos3 = wheelThree.getCurrentPosition();
        double pos4 = wheelFour.getCurrentPosition();
        double avgpos = 0;
        double timeStart=currentOpMode.getRuntime();
        double angleDiff=0;
        double rotation;
        angleDrive(currentOpMode,speed,angle,0);
        while (currentOpMode.opModeIsActive() && avgpos <= targetTicks && (currentOpMode.getRuntime()-timeStart)<timeLimit) {
            currentOpMode.telemetry.addData("targetTicks", targetTicks);
            currentOpMode.telemetry.addData("avgpos", avgpos);
            currentOpMode.telemetry.update();
            avgpos = Math.abs(wheelOne.getCurrentPosition() - pos1) + Math.abs(wheelTwo.getCurrentPosition() - pos2) + Math.abs(wheelThree.getCurrentPosition() - pos3) + Math.abs(wheelFour.getCurrentPosition() - pos4);
            avgpos = avgpos / 4;
            angleDiff=gyroTarget-gyroAngle(gyro.getIntegratedZValue());
            if (angleDiff<180) angleDiff = 360+angleDiff;
            if (angleDiff>180) angleDiff = angleDiff-360;
            rotation = -angleDiff / 25;
            rotation=Range.clip(rotation,-0.2,0.2);


            angleDrive(currentOpMode, speed, angle, rotation);

            currentOpMode.telemetry.addData("Heading", gyroTarget);
            currentOpMode.telemetry.addData("rotation", rotation);
            currentOpMode.telemetry.addData("Encoders",wheelOne.getCurrentPosition() + "||" +wheelTwo.getCurrentPosition() + "||" + wheelThree.getCurrentPosition() + "||" +wheelFour.getCurrentPosition());
            currentOpMode.telemetry.addData("wheelPowers",wheelOne.getPower() + "||" +wheelTwo.getPower() + "||" + wheelThree.getPower() + "||" +wheelFour.getPower());
            currentOpMode.telemetry.addData("wheelStatus",wheelOne.isBusy() + "||" +wheelTwo.isBusy() + "||" + wheelThree.isBusy() + "||" +wheelFour.isBusy());


            currentOpMode.idle();
        }
        stopRobot();
    }


    public void gyroTurn(LinearOpMode currentOpMode, double rotationSpeed,double targetAngle,boolean counterClock){

        if( counterClock) {
            while (currentOpMode.opModeIsActive() && (gyroAngle(gyro.getIntegratedZValue())) < targetAngle) {
                angleDrive(currentOpMode, 0, 0, -rotationSpeed);
                currentOpMode.telemetry.addData("robotAngle", gyroAngle(gyro.getIntegratedZValue()));
                currentOpMode.telemetry.update();
                currentOpMode.idle();
            }
        }
        else{
            while (currentOpMode.opModeIsActive() && (gyroAngle(gyro.getIntegratedZValue())) > targetAngle) {
                angleDrive(currentOpMode, 0, 0, rotationSpeed);
                currentOpMode.telemetry.addData("robotAngle", gyroAngle(gyro.getIntegratedZValue()));
                currentOpMode.telemetry.update();
                currentOpMode.idle();
            }

        }
        stopRobot();
    }

    public void releaseLowerCubeArm(){
        CubeClawRight.setPosition(0.7);
        CubeClawLeft.setPosition(0.3);
        try {
            Thread.sleep(200);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        CubeLiftArm.setPosition(CubeLiftArmDownPosition);

    }
    public void GemGame(OpMode currentOpMode,boolean redSide){
        GemExtender.setPower(GemExtenderLastPosition-0.2);
        // GemRotater.setPosition(GemRotatorInitialPosition-0.1);

        CubeLiftArm.setPosition(CubeLiftArmUpPosition); // lift the cube lift arm to the up position

        try {
            Thread.sleep(2000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        double redFirst1 = (double) sensorRGB.red();
        double blueFirst1 = (double)sensorRGB.blue();

        CubeClawRight.setPosition(0);
        CubeClawLeft.setPosition(1);

        GemExtender.setPower(GemExtenderLastPosition-0.1);
        // CubeSecondClaw.setPosition(cubeSecondOpenPos);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        CubeSecondClaw.setPosition(cubeSecondClosePos);
        double redFirst2 = (double)sensorRGB.red();
        double blueFirst2 = (double)sensorRGB.blue();



        GemExtender.setPower(GemExtenderMiddlePosition);
        try {
            Thread.sleep(500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }


        GemRotater.setPosition(GemRotatorMidPosition);
        try {
            Thread.sleep(300);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }
        GemExtender.setPower(GemExtenderLastPosition-0.2);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        double redSecond1 = (double)sensorRGB.red();
        double blueSecond1 = (double)sensorRGB.blue();

        GemExtender.setPower(GemExtenderLastPosition);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }


        double redSecond2 = (double)sensorRGB.red();
        double blueSecond2 = (double)sensorRGB.blue();





        double ratioDiffFirst=(redFirst2-redFirst1)/(blueFirst2-blueFirst1+1);

        double ratioDiffSecond=(redSecond2-redSecond1)/(blueSecond2-blueSecond1+1);
        currentOpMode.telemetry.addData("ratiosDiffFirst",(redFirst2-redFirst1)/(blueFirst2-blueFirst1+1));
        currentOpMode.telemetry.addData("ratiosDiffSecond",(redSecond2-redSecond1)/(blueSecond2-blueSecond1+1));
        currentOpMode.telemetry.addData("redSide",redSide);
        currentOpMode.telemetry.update();

        if(blueFirst1>redFirst1 || blueFirst2>redFirst2){
            if(redSide)GemRotater.setPosition(GemRotatorInitialPosition-0.1);
            else GemRotater.setPosition(GemRotatorLastPosition+0.1);

        }
        else if(blueSecond1>redSecond1 || blueSecond2>redSecond2){
            if(redSide) GemRotater.setPosition(GemRotatorLastPosition+0.1);
            else GemRotater.setPosition(GemRotatorInitialPosition-0.1);

        }
        // between the two readings, use the one with larger difference between blue and red
        else if(Math.abs(ratioDiffFirst-ratioDiffSecond)>1 ){
            if((ratioDiffFirst > ratioDiffSecond) && !(ratioDiffFirst>1 && ratioDiffSecond>1)&& !(ratioDiffFirst<0 && ratioDiffSecond<0)) {

                if(redSide) GemRotater.setPosition(GemRotatorLastPosition+0.1);
                else GemRotater.setPosition(GemRotatorInitialPosition-0.1);
            }


            else if(ratioDiffFirst<ratioDiffSecond){


                if(redSide)GemRotater.setPosition(GemRotatorInitialPosition-0.1);
                else GemRotater.setPosition(GemRotatorLastPosition+0.1);
            }


        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        //GemRotater.setPosition(GemRotatorMidPosition);
        GemExtender.setPower(GemExtenderInitialPosition);

        try {
            Thread.sleep(1500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        GemRotater.setPosition(GemRotatorInitialPosition);
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);


        // turn the LED off
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }

    public void ReleaseCube(LinearOpMode currentOpMode){
        double timeStart;

        CubeSecondClaw.setPosition(cubeSecondOpenPos);
        CubeClawRight.setPosition(0.7);
        CubeClawLeft.setPosition(0.3);
        try {
            Thread.sleep(500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        timeStart=currentOpMode.getRuntime();


        while(currentOpMode.opModeIsActive() &&(currentOpMode.getRuntime()-timeStart)<0.5) {
            angleDrive(currentOpMode, 0.20, 180, 0);
            currentOpMode.idle();

        }
        stopRobot();
        try {
            Thread.sleep(700);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        timeStart=currentOpMode.getRuntime();
        while(currentOpMode.opModeIsActive() &&(currentOpMode.getRuntime()-timeStart)<0.7) {
            angleDrive(currentOpMode, 0.20, 180, 0);
            currentOpMode.idle();

        }
        stopRobot();

        timeStart=currentOpMode.getRuntime();

        while(currentOpMode.opModeIsActive() &&(currentOpMode.getRuntime()-timeStart)<1.5) {
            angleDrive(currentOpMode, 0.20, 0, 0);
            currentOpMode.idle();

        }
        stopRobot();
        timeStart=currentOpMode.getRuntime();

        while(currentOpMode.opModeIsActive() &&(currentOpMode.getRuntime()-timeStart)<1.5) {
            angleDrive(currentOpMode, 0.20, 180, 0);
            currentOpMode.idle();

        }
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