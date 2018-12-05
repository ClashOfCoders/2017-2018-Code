package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;


        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="NavXTeleOpTest", group="Iterative Opmode")
//@Disabled
public class NavXTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    boolean initialDirectionSet=false;
    boolean CubeSecondOpen=true;
    boolean CubeSecondLifted=false;
    boolean runOnce=false;

    double CubeLiftUpPower = 0.8;

    // Toggle between open and closed state
    boolean CubeClawOpen=true;
    boolean RelicClawOpen=true;
    double CubeClawPosition=0;
    double CubeLiftArmPosition=0.1;
    private  int lastMotorDirection=1;
    private  int lastRelicLiftDirection=1;

    double relicClawLeft=0;
    double relicClawRight=1;
    double relicLiftPos=0.5;
    double ClawLiftPos = 0;
    boolean ClawLiftChange=true;
    private double gyroTarget=0;


    private HardwareRelic robot = new HardwareRelic();
    private boolean relicPosition = true;
    private boolean CubeClawOpenLeft=true;
    private double RelicLiftPos=0;
    private boolean rotationState=false;
    private double zeroheading=0;
    private double angleDiff=0;
    double targetPosition=-1900;
    boolean higerToLower=true;
    private int driverState;
    private double timeStampCubeClaw=0;
    private double timeStampCubeSecond=0;
    private double timeStampRelicClaw=0;
    private double timeStampCubeLiftArm=0;
    private double timeStampPassUpCube=0;
    private int passUpCubeStep=1;
    static final double kOffBalanceAngleThresholdDegrees = 3.0f;
    static final double kOnBalanceAngleThresholdDegrees  = 3.0f;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private boolean calibration_complete = false;
    private boolean autoBalanceXMode = false;
    private boolean autoBalanceYMode = false;
    private double xAxisRate;
    private double yAxisRate;
    VisionClassInstance trackVision;
    private double speed=0;
    int XControl=0;
    int YControl=0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        trackVision = new VisionClassInstance();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        trackVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
         trackVision.setShowVision(false);
        // start the vision system
         trackVision.enable();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.setDriveEncoderMode(true);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        //telemetry.log().add("NavXGyro Calibrating. Do Not Move!");
        // Wait until the gyro calibration is complete
        runtime.reset();
        while (robot.navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();

        }
        telemetry.log().clear(); telemetry.log().add("NavX Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();




    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        while( !robot.RelicSwitch.isPressed()){

            robot.RelicLifter.setPower(0.4);
            runOnce=true;


        }


        robot.RelicLifter.setPower(0);
        robot.RelicLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RelicLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        AngularVelocity rates = robot.gyroN.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = robot.gyroN.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);




        double angle;
        double speed;

        double x; double y;
        double stickAngle=robot.getStickAngle(gamepad1.left_stick_x,gamepad1.left_stick_y);

        if (Math.abs(gamepad1.left_stick_x)<Math.abs(gamepad1.left_stick_y)){
            x=0;
            y=gamepad1.left_stick_y;

        }
        else {
            y=0;
            x=gamepad1.left_stick_x;
        }



        angle = robot.getAngle(x,y);
        speed = Math.sqrt(x * x + y*y);
        double rotation;


        if(gamepad1.left_bumper ){
            zeroheading=angles.firstAngle;
            driverState=0;

        }
        telemetry.addData("zeroHeading set as",zeroheading);
        telemetry.addData("gyroheading is",angles.firstAngle);


        if(gamepad1.right_stick_x == 0 &&(gamepad1.left_stick_y !=0 || gamepad1.left_stick_x!=0) &&runtime.milliseconds()<119990 ) {

            if(rotationState){
                gyroTarget=angles.firstAngle;
                rotationState=false;
            }
            angleDiff=gyroTarget-angles.firstAngle;;
            if(angleDiff>180) angleDiff=angleDiff-360;
            else if(angleDiff<-180) angleDiff=360+angleDiff;
            rotation=-angleDiff/25;
            rotation=Range.clip(rotation,-0.2,0.2);

            double newAngle=angle;
            double angleShift=angles.firstAngle-zeroheading;
            if(angleShift>180) angleShift=angleShift-360;
            if(angleShift<-180) angleShift=360+angleShift;

            double relativeAngle=stickAngle-angleShift;
            if(relativeAngle>180) relativeAngle=relativeAngle-360;
            if(relativeAngle<-180) relativeAngle=360+relativeAngle;

            relativeAngle=-relativeAngle;

            newAngle=Math.round(relativeAngle/90)*90;
            if(newAngle<0) newAngle +=360;



            speed=speed*0.75;
            if (gamepad1.right_trigger>0)
                speed/=2;

            robot.angleDrive(this, speed, newAngle,  rotation);



            //telemetry.addData("leftstick",gamepad1.left_stick_x+ "-" +gamepad1.left_stick_y);
            //telemetry.addData("RightStick",gamepad1.right_stick_x);
            telemetry.addData("angle", angle);
            telemetry.addData("angleShift",angleShift);
            telemetry.addData("newAngle", newAngle);
            telemetry.addData("DriverState",driverState);
            //  telemetry.addData("rotation",rotation);
            //  telemetry.addData("Power 1-2-3-4", robot.wheelOne.getPower() + "-" + robot.wheelTwo.getPower() + "-" + robot.wheelThree.getPower() + "-" + robot.wheelFour.getPower());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.update();




            //gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());
        }
        else if(gamepad1.right_stick_x!=0 && gamepad1.left_stick_y==0 && gamepad1.left_stick_x==0 ){
            rotationState=true;
            rotation=gamepad1.right_stick_x;

            speed=0;
            rotation*=0.9;

            if (gamepad1.right_trigger>0) {
                speed /= 2;
                rotation /= 2;
            }

            // robot.angleDrive(this, speed/3, angle, (double) gamepad1.right_stick_x/3);
            robot.angleDrive(this, speed, angle, rotation);
            gyroTarget=angles.firstAngle;
           /*
            telemetry.addData("leftstick",gamepad1.left_stick_x+ "-" +gamepad1.left_stick_y);
            telemetry.addData("RightStick",gamepad1.right_stick_x);
            telemetry.addData("angle", angle);
            telemetry.addData("rotation",rotation);
            telemetry.addData("Power 1-2-3-4", robot.wheelOne.getPower() + "-" + robot.wheelTwo.getPower() + "-" + robot.wheelThree.getPower() + "-" + robot.wheelFour.getPower());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            */


        }
        else if(gamepad1.right_stick_x==0 && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0){
            robot.setSpeed(this,0,0,0,0);
            gyroTarget=angles.firstAngle;

        }






        /****************************************************
         * Code for handeling Cube Liner Slide and Cube Lift
         ****************************************************/

        if(gamepad2.dpad_left && !(robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed()) && runtime.seconds()<120) {
            robot.CubeTimeBelt.setPower(1);
        }
        else if(gamepad2.dpad_right && !(robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed()) && runtime.seconds()<120) {
            robot.CubeTimeBelt.setPower(-1);
        }
        else if(((!gamepad2.dpad_right && !gamepad2.dpad_left) || (robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed())) && runtime.seconds()<120)
        {
            robot.CubeTimeBelt.setPower(0);
        }

        //  telemetry.addData("Switch R/ Switch L", robot.CubeSlideSwitchR.isPressed() + " / " + robot.CubeSlideSwitchL.isPressed());
        // telemetry.addData("dpadLeft / dpadRight / ", gamepad2.dpad_left + " / " + gamepad2.dpad_right);



        if(robot.CubeSlideSwitchR.isPressed()) {
            while (robot.CubeSlideSwitchR.isPressed() && runtime.seconds()<120)
                robot.CubeTimeBelt.setPower(1);
            robot.CubeTimeBelt.setPower(0);
        }
        if(robot.CubeSlideSwitchL.isPressed()) {
            while (robot.CubeSlideSwitchL.isPressed() && runtime.seconds()<120)
                robot.CubeTimeBelt.setPower(-1);
            robot.CubeTimeBelt.setPower(0);
        }

/*
        if(gamepad1.dpad_up && (!robot.CubeLiftSwitch.isPressed()) && runtime.seconds()<150){
            robot.CubeLift1.setPower(CubeLiftUpPower);
            robot.CubeLift2.setPower(CubeLiftUpPower);
        }
        else if(gamepad1.dpad_down && (!robot.CubeLiftSwitch.isPressed()) && runtime.seconds()<150) {
            robot.CubeLift1.setPower(-CubeLiftUpPower);
            robot.CubeLift2.setPower(-CubeLiftUpPower);
        }
        else if(((!gamepad1.dpad_up && !gamepad1.dpad_down)  ) )
        {
            robot.CubeLift1.setPower(0);
            robot.CubeLift2.setPower(0);
        }
        if(robot.CubeLiftSwitch.isPressed()) {
            CubeLiftUpPower=robot.CubeLift1.getPower();
            while (robot.CubeLiftSwitch.isPressed() && runtime.seconds()<150) {
                robot.CubeLift1.setPower(-CubeLiftUpPower);
                robot.CubeLift2.setPower(-CubeLiftUpPower);
            }
            robot.CubeLift1.setPower(0);
            robot.CubeLift2.setPower(0);
        }
        */


        if (gamepad2.dpad_up) {
            if ((!robot.CubeLiftSwitch.isPressed() || (robot.CubeLiftSwitch.isPressed() && lastMotorDirection == -1)) && runtime.seconds() < 120) {
                robot.CubeLift1.setPower(0.8);
                robot.CubeLift2.setPower(0.8);
                if(!robot.CubeLiftSwitch.isPressed())
                    lastMotorDirection=1;
            }
            else if(robot.CubeLiftSwitch.isPressed() && lastMotorDirection==1 ){
                robot.CubeLift1.setPower(0);
                robot.CubeLift2.setPower(0);
            }
        }

        if (gamepad2.dpad_down){
            if( (!robot.CubeLiftSwitch.isPressed() || (robot.CubeLiftSwitch.isPressed() && lastMotorDirection == 1)) && runtime.seconds() < 120) {

                robot.CubeLift1.setPower(-0.8);
                robot.CubeLift2.setPower(-0.8);
                if(!robot.CubeLiftSwitch.isPressed())
                    lastMotorDirection=-1;
            }
            else if(robot.CubeLiftSwitch.isPressed() && lastMotorDirection==-1){
                robot.CubeLift1.setPower(0);
                robot.CubeLift2.setPower(0);

            }
        }
        if (!gamepad2.dpad_up && !gamepad2.dpad_down){
            robot.CubeLift1.setPower(0);
            robot.CubeLift2.setPower(0);
        }


        telemetry.addData("Position of CubeTimeBelt", robot.CubeTimeBelt.getPower());


        if(gamepad2.x  && runtime.seconds()<120 && (runtime.milliseconds()-timeStampCubeClaw)>300) {



            CubeClawPosition +=1;
            timeStampCubeClaw=runtime.milliseconds();



            CubeClawPosition = Range.clip(CubeClawPosition, 0, 2);
            if (CubeClawPosition==0){
                robot.CubeClawRight.setPosition(1);
                robot.CubeClawLeft.setPosition(0);

            }
            else if(CubeClawPosition==1){
                robot.CubeClawRight.setPosition(0.3);
                robot.CubeClawLeft.setPosition(0.7);

            }
            else if(CubeClawPosition==2){
                robot.CubeClawRight.setPosition(0);
                robot.CubeClawLeft.setPosition(1);
            }
        }
        if(gamepad2.b && runtime.seconds()<120 && (runtime.milliseconds()-timeStampCubeClaw)>300) {




            CubeClawPosition -=1;
            timeStampCubeClaw=runtime.milliseconds();

            CubeClawPosition = Range.clip(CubeClawPosition, 0, 2);
            if (CubeClawPosition==0){
                robot.CubeClawRight.setPosition(1);
                robot.CubeClawLeft.setPosition(0);

            }
            else if(CubeClawPosition==1){
                robot.CubeClawRight.setPosition(0.3);
                robot.CubeClawLeft.setPosition(0.7);

            }
            else if(CubeClawPosition==2){
                robot.CubeClawRight.setPosition(0);
                robot.CubeClawLeft.setPosition(1);
            }
        }







        /*code for second cube claw*/
        if(gamepad2.right_bumper && runtime.seconds() < 120 && (runtime.milliseconds()-timeStampCubeSecond)>200){
            timeStampCubeSecond=runtime.milliseconds();
            if(CubeSecondOpen==true) {
                robot.CubeSecondClaw.setPosition(robot.cubeSecondClosePos);
                CubeSecondOpen = false;
            }
            else {
                robot.CubeSecondClaw.setPosition(robot.cubeSecondOpenPos);
                CubeSecondOpen = true;
            }

        }

    /*Code for CubeLiftArm Control*/
        if(gamepad2.y   && runtime.seconds()<120 && (runtime.milliseconds()-timeStampCubeLiftArm)>200) {
            timeStampCubeLiftArm=runtime.milliseconds();


            CubeLiftArmPosition +=0.3;



            CubeLiftArmPosition = Range.clip(CubeLiftArmPosition, 0.1, 1);
            robot.CubeLiftArm.setPosition(CubeLiftArmPosition);

        }
        if( gamepad2.a && runtime.seconds()<120 && (runtime.milliseconds()-timeStampCubeLiftArm)>200) {
            timeStampCubeLiftArm=runtime.milliseconds();


            CubeLiftArmPosition -=0.3;

            CubeLiftArmPosition = Range.clip(CubeLiftArmPosition, 0.1, 1);
            robot.CubeLiftArm.setPosition(CubeLiftArmPosition);

        }
        if(gamepad2.right_trigger>0 &&runtime.seconds()<120){

            if(passUpCubeStep==1 && (runtime.milliseconds()-timeStampPassUpCube)>1500){
                passUpCubeStep=2;

                robot.CubeSecondClaw.setPosition(robot.cubeSecondOpenPos);

                CubeClawPosition = 2;
                timeStampPassUpCube=runtime.milliseconds();


                robot.CubeClawRight.setPosition(0);
                robot.CubeClawLeft.setPosition(1);


            }

            if(passUpCubeStep==2 && (runtime.milliseconds()-timeStampPassUpCube)>600){
                passUpCubeStep=3;
                timeStampPassUpCube=runtime.milliseconds();
                robot.CubeLiftArm.setPosition(robot.CubeLiftArmUpPosition);

            }
            if(  passUpCubeStep==3 && (runtime.milliseconds()-timeStampPassUpCube)>600){
                passUpCubeStep=4;
                timeStampPassUpCube=runtime.milliseconds();
                robot.CubeSecondClaw.setPosition(robot.cubeSecondClosePos);

            }
            if( passUpCubeStep==4 && (runtime.milliseconds()-timeStampPassUpCube)>300){
                passUpCubeStep=5;
                timeStampPassUpCube=runtime.milliseconds();
                CubeClawPosition = 1;
                CubeClawOpen=true;

                robot.CubeClawRight.setPosition(0.3);
                robot.CubeClawLeft.setPosition(0.7);



            }
            if( passUpCubeStep==5 && (runtime.milliseconds()-timeStampPassUpCube)>600){
                passUpCubeStep=1;
                timeStampPassUpCube=runtime.milliseconds();
                robot.CubeLiftArm.setPosition(robot.CubeLiftArmDownPosition);
                CubeSecondOpen = false;

            }

        }


       /* if(gamepad2.left_trigger > 0 ){
            robot.CubeClawRight.setPosition(0.3);
            robot.CubeClawLeft.setPosition(0.7);
            CubeClawOpen=true;
            try {
                Thread.sleep(200);
                } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            robot.CubeLiftArm.setPosition(robot.CubeLiftArmUpPosition);
            try {
                Thread.sleep(400);

            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            robot.CubeClawRight.setPosition(0);
            robot.CubeClawLeft.setPosition(1);
            CubeClawOpen=false;
            try {
                Thread.sleep(200);
                } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            robot.CubeSecondClaw.setPosition(robot.cubeSecondOpenPos);
            CubeSecondOpen=true;
            try {
                Thread.sleep(200);
                } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            robot.CubeLiftArm.setPosition(robot.CubeLiftArmDownPosition+0.3);
            try {
                Thread.sleep(200);
                } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }


        }*/





       /* Code for dealing with the Relic Claw*/

        if(gamepad2.left_bumper && runtime.seconds()<120 && (runtime.milliseconds()-timeStampRelicClaw)>400){
            timeStampRelicClaw=runtime.milliseconds();
            if (RelicClawOpen){
                robot.RelClaw.setPosition(0.4);
                RelicClawOpen=false;
            }
            else{

                robot.RelClaw.setPosition(0);
                RelicClawOpen=true;
            }
        }







        if (gamepad2.left_stick_y<0 && runtime.seconds()<120) {
            if ((!robot.RelicSwitch.isPressed() || (robot.RelicSwitch.isPressed() && lastRelicLiftDirection == -1)) && runtime.seconds() < 150) {
                robot.RelicLifter.setPower(1);
                if(!robot.RelicSwitch.isPressed())
                    lastRelicLiftDirection=1;
            }
            else if(robot.RelicSwitch.isPressed() && lastRelicLiftDirection==1 ){
                robot.RelicLifter.setPower(0);
            }
            telemetry.addData("currentRelicLiftPosition",robot.RelicLifter.getCurrentPosition());
            telemetry.addData("targetPosition",targetPosition);
            telemetry.addData("higherToLower",higerToLower);
            telemetry.update();
        }

        if (gamepad2.left_stick_y>0 && runtime.seconds()<120){
            if( (!robot.RelicSwitch.isPressed() || (robot.RelicSwitch.isPressed() && lastRelicLiftDirection == 1))) {

                robot.RelicLifter.setPower(-1);

                if(!robot.RelicSwitch.isPressed())
                    lastRelicLiftDirection=-1;
            }
            else if(robot.RelicSwitch.isPressed() && lastRelicLiftDirection==-1){
                robot.RelicLifter.setPower(0);

            }
            telemetry.addData("currentRelicLiftPosition",robot.RelicLifter.getCurrentPosition());
            telemetry.addData("targetPosition",targetPosition);
            telemetry.addData("higherToLower",higerToLower);
            telemetry.update();
        }
        if (gamepad2.left_stick_y==0 && gamepad2.left_trigger==0){
            robot.RelicLifter.setPower(0);
        }


        if(gamepad2.left_trigger>0 && gamepad2.left_stick_y==0){

            double Diff=(robot.RelicLifter.getCurrentPosition()-targetPosition);
            double DiffSpeed=-Diff/2000;

            if(  Math.abs(Diff)>50)  {


                robot.RelicLifter.setPower(DiffSpeed);
                telemetry.addData("currentRelicLiftPosition",robot.RelicLifter.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("currentRelicLiftPosition",robot.RelicLifter.getCurrentPosition());
            telemetry.addData("targe tPosition",targetPosition);
            telemetry.addData("Diff",Diff);
            telemetry.update();



        }


        if (gamepad2.right_stick_x!=0 && runtime.seconds()<120) {
            if (gamepad2.right_stick_x > 0)
                robot.RelicExtender.setPower(-0.5);
            else if (gamepad2.right_stick_x < 0)
                robot.RelicExtender.setPower(0.5);
        }
        if(gamepad2.right_stick_x==0 || runtime.seconds()>120)
            robot.RelicExtender.setPower(0);

        if(gamepad1.x) {
            double pitchAngleDegrees = angles.thirdAngle;
            double rollAngleDegrees = angles.secondAngle;

            if (!autoBalanceYMode &&
                    (Math.abs(rollAngleDegrees) >=
                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = true;
            } else if (autoBalanceYMode &&
                    (Math.abs(rollAngleDegrees) <=
                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = false;
            }
            if (!autoBalanceXMode &&
                    (Math.abs(pitchAngleDegrees) >=
                            Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = true;
            } else if (autoBalanceXMode &&
                    (Math.abs(pitchAngleDegrees) <=
                            Math.abs(kOnBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = false;
            }

            // Control drive system automatically,
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle

            if (autoBalanceXMode ) {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                xAxisRate = Math.sin(pitchAngleRadians) * 3;
                // xAxisRate=Range.clip(-0.4,0.4,xAxisRate);

                robot.angleDrive(this,xAxisRate,0, 0);


            }


            if (autoBalanceYMode  ) {
                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
                yAxisRate = Math.sin(rollAngleRadians) * -3;
                // yAxisRate=Range.clip(-0.4,0.4,yAxisRate);
                robot.angleDrive(this,yAxisRate,270, 0);
            }

            if(!autoBalanceYMode && !autoBalanceXMode){
                robot.stopRobot();
            }


            // At this point, the X/Axis motion rates are proportional to the
            // angle, and in the inverse direction.

            // NOTE:  This algorithm assumes an omni-directional drive system (e.g., Mecanum)
            // that can navigate linearly in both X and Y axis direction.  Tank-style drive
            // systems (without the ability to travel in a linear direction in the "strafe"
            // [side-to-side] direction will require additional logic.
            telemetry.addData("Pitch Angle (degrees):", pitchAngleDegrees);
            telemetry.addData("Roll Angle (degrees): ", rollAngleDegrees);
            telemetry.addData("X Axis Balance Rate:  ", xAxisRate);
            telemetry.addData("Y Axis Balance Rate:  ", yAxisRate);
            telemetry.addData("Y Balance State", autoBalanceYMode);


        }


        if(runtime.seconds()>120)
            robot.stopRobot();



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopRobot();
    }

}
