package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name="Basic: Iterative relicOpMode", group="Iterative Opmode")
//@Disabled
public class teleoprelic extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    double CubeLiftUpPower = 0.8;

    // Toggle between open and closed state
    boolean CubeClawOpen=true;
    boolean RelicClawOpen=true;
    double CubeLiftArmPosition=0.9;
    private  int lastMotorDirection=1;

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
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.setDriveEncoderMode(true);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        robot.gyro.calibrate();



        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.relicClawLifter.setPower(0);
        if (robot.gyro.isCalibrating()) {
            telemetry.addData(">", "Calibrating Gyro");    //
            telemetry.update();
        }
        else
        {
            telemetry.addData(">", "Gyro Calibrated");    //
            telemetry.update();
            gyroTarget = robot.gyroAngle(robot.gyro.getIntegratedZValue());
        }

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
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





        double angle;
        double speed;

        double x; double y;
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

        if(gamepad1.right_stick_x == 0 &&(gamepad1.left_stick_y !=0 || gamepad1.left_stick_x!=0) &&runtime.milliseconds()<149990 ) {

            if(rotationState){
                gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());
                rotationState=false;
            }

            rotation=-(gyroTarget-robot.gyroAngle(robot.gyro.getIntegratedZValue()))/25;

            speed=speed*0.75;
            if (gamepad1.right_trigger>0)
                speed/=2;

            // robot.angleDrive(this, speed/3, angle, (double) gamepad1.right_stick_x/3);
            robot.angleDrive(this, speed, angle,  rotation);


            telemetry.addData("leftstick",gamepad1.left_stick_x+ "-" +gamepad1.left_stick_y);
            telemetry.addData("RightStick",gamepad1.right_stick_x);
            telemetry.addData("angle", angle);
            telemetry.addData("rotation",rotation);
            telemetry.addData("Power 1-2-3-4", robot.wheelOne.getPower() + "-" + robot.wheelTwo.getPower() + "-" + robot.wheelThree.getPower() + "-" + robot.wheelFour.getPower());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());




            //gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());
        }
        else if(gamepad1.right_stick_x!=0 && gamepad1.left_stick_y==0 && gamepad1.left_stick_x==0 ){
            rotationState=true;
            rotation=gamepad1.right_stick_x;

            speed *=0.75;
            rotation*=0.75;

            if (gamepad1.right_trigger>0) {
                speed /= 2;
                rotation /= 2;
            }

            // robot.angleDrive(this, speed/3, angle, (double) gamepad1.right_stick_x/3);
            robot.angleDrive(this, speed, angle, rotation);
            gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());
            telemetry.addData("leftstick",gamepad1.left_stick_x+ "-" +gamepad1.left_stick_y);
            telemetry.addData("RightStick",gamepad1.right_stick_x);
            telemetry.addData("angle", angle);
            telemetry.addData("rotation",rotation);
            telemetry.addData("Power 1-2-3-4", robot.wheelOne.getPower() + "-" + robot.wheelTwo.getPower() + "-" + robot.wheelThree.getPower() + "-" + robot.wheelFour.getPower());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


        }
        else if(gamepad1.right_stick_x==0 && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0){
            robot.setSpeed(this,0,0,0,0);
            gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());

        }


        //robot.angleDrive(this, speed, angle, gamepad1.right_stick_x/2.5);




            /****************************************************
             * Code for handeling Cube Liner Slide and Cube Lift
             ****************************************************/

        if(gamepad2.dpad_left && !(robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed()) && runtime.seconds()<150) {
            robot.CubeTimeBelt.setPower(1);
        }
        else if(gamepad2.dpad_right && !(robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed()) && runtime.seconds()<150) {
            robot.CubeTimeBelt.setPower(-1);
        }
        else if(((!gamepad2.dpad_right && !gamepad2.dpad_left) || (robot.CubeSlideSwitchR.isPressed() || robot.CubeSlideSwitchL.isPressed())) && runtime.seconds()<150)
        {
            robot.CubeTimeBelt.setPower(0);
        }

        telemetry.addData("Switch R/ Switch L", robot.CubeSlideSwitchR.isPressed() + " / " + robot.CubeSlideSwitchL.isPressed());
        telemetry.addData("dpadLeft / dpadRight / ", gamepad2.dpad_left + " / " + gamepad2.dpad_right);



        if(robot.CubeSlideSwitchR.isPressed()) {
            while (robot.CubeSlideSwitchR.isPressed() && runtime.seconds()<150)
                robot.CubeTimeBelt.setPower(1);
            robot.CubeTimeBelt.setPower(0);
        }
        if(robot.CubeSlideSwitchL.isPressed()) {
            while (robot.CubeSlideSwitchL.isPressed() && runtime.seconds()<150)
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
            if ((!robot.CubeLiftSwitch.isPressed() || (robot.CubeLiftSwitch.isPressed() && lastMotorDirection == -1)) && runtime.seconds() < 150) {
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
            if( (!robot.CubeLiftSwitch.isPressed() || (robot.CubeLiftSwitch.isPressed() && lastMotorDirection == 1)) && runtime.seconds() < 150) {

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


        /*Code for get Cubs using CubeClaws */
        if(gamepad2.x && runtime.seconds()<150){
            if (CubeClawOpen){
                robot.CubeClawRight.setPosition(0);
                robot.CubeClawLeft.setPosition(1);
                CubeClawOpen=false;
            }
            else{
                //robot.CubeClawRight.setPosition(0.6);
               // robot.CubeClawLeft.setPosition(0.4);
                robot.CubeClawRight.setPosition(0.3);
                 robot.CubeClawLeft.setPosition(0.7);
                CubeClawOpen=true;
            }
            try {
                Thread.sleep(200); /* wait for 0.2 second to avoid double reading of the same button press*/
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

        }
       // set the b button for small release
        if(gamepad2.b && runtime.seconds()<150){
            /*
            if (CubeClawOpenLeft){

                robot.CubeClawLeft.setPosition(1);
                CubeClawOpenLeft=false;
            }
            else{

                robot.CubeClawLeft.setPosition(0.4);
                CubeClawOpenLeft=true;
            }
            */
            robot.CubeClawRight.setPosition(0.15);
            robot.CubeClawLeft.setPosition(0.85);



        }

        /*Code for CubeLiftArm Control*/
        if(gamepad2.y  || gamepad2.a && runtime.seconds()<150) {

            if(gamepad2.a)
                CubeLiftArmPosition =0.8;

            else if (gamepad2.y)
                CubeLiftArmPosition -=0;

            CubeLiftArmPosition = Range.clip(CubeLiftArmPosition, 0, 0.8);
            robot.CubeLiftArm.setPosition(CubeLiftArmPosition);
        }





       /* Code for dealing with the Relic Claw*/

        if(gamepad2.left_bumper && runtime.seconds()<150){
            if (RelicClawOpen){
                robot.RelClaw.setPosition(0.4);
                RelicClawOpen=false;
            }
            else{

                robot.RelClaw.setPosition(0);
                RelicClawOpen=true;
            }
            try {
                Thread.sleep(200); /* wait for 0.2 second to avoid double reading of the same button press*/
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }


        }




      /* if (gamepad2.right_bumper){
            ClawLiftChange = !ClawLiftChange;
            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }
        if (ClawLiftChange)
            ClawLiftPos = -gamepad2.left_stick_y;

        Range.clip(ClawLiftPos, -0.7, 0.55);
        robot.relicClawLifter.setPower(ClawLiftPos);

        telemetry.addData("Position of relicClawLifter", robot.relicClawLifter.getPower());
        telemetry.addData("Can relicLift move (press RB to change)", ClawLiftChange);

        */




        if(gamepad2.left_stick_y!=0 && runtime.seconds()<150) {

            if (gamepad2.left_stick_y < 0 ) {
                RelicLiftPos +=0.02;
                RelicLiftPos=Range.clip(RelicLiftPos,-0.75,0.55);
                robot.relicClawLifter.setPower(RelicLiftPos);
            }
            if (gamepad2.left_stick_y > 0 )
            RelicLiftPos -=0.02;
            RelicLiftPos=Range.clip(RelicLiftPos,-0.7,0.55);
            robot.relicClawLifter.setPower(RelicLiftPos);
            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }



        if (gamepad2.right_stick_x!=0 && runtime.seconds()<150) {
            if (gamepad2.right_stick_x > 0)
                robot.RelicExtender.setPosition(0);
            else if (gamepad2.right_stick_x < 0)
                robot.RelicExtender.setPosition(1);
        }
        if(gamepad2.right_stick_x==0 || runtime.seconds()>150)
            robot.RelicExtender.setPosition(0.52);



        if (gamepad1.a) {
            robot.GemExtender.setPosition(1);
            telemetry.addData("gamepad1", "a");
        }else if (gamepad1.b) {
            robot.GemExtender.setPosition(0);
            telemetry.addData("gamepad1", "b");
        }


        if (gamepad1.x){
            robot.GemRotater.setPosition(0);


        }
        else if (gamepad1.y){
            robot.GemRotater.setPosition(0.9);

        }




        if(runtime.seconds()>150)
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