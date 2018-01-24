package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by jianqiuzhang on 12/12/17.
 */
@Autonomous(name="RedFront", group ="Concept")
// @Disabled
public class RedFront extends LinearOpMode {
    // VuforiaMan vuManager = new VuforiaMan();
    //  VuforiaResults vuforiaResults = new VuforiaResults();
    public HardwareRelic robot = new HardwareRelic();
    VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    final int CAMERA_FORWARD_DISPLACEMENT = 110;   // Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT = 0;     // Camera is ON the robots center line
    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZX,
                    AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90, 0, 0));

    OpenGLMatrix targetOrientation = OpenGLMatrix
            .translation(0, 0, 150)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XYZ,
                    AngleUnit.DEGREES, 90, 0, -90));

    public void runOpMode() {
        robot.init(hardwareMap);
           /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Ac8eIir/////AAAAGaSY6aHHDUtrtDd/zYyIzTxjCvXt3xN6sh7gNQHEMhMJtoo/1ELPhrQYsu5gnkwUeCzwQvfdKnU8mbTJNIA3OszGg4wAd8in1Up/bsxi84WseRf0mNiPzKzpgrPQqodfBfyDnlxOK9/XU3eZ/wJO2jUvD6l4tO5GPOXxkxch5N5er/43UC4IDe5Lynzrkv5Y0dEsrU+7OzXvqvYHJcDlknvh1uA+42BOEPOTFUQQoQ6/sgy/x0+m5zcsCh7vFUPbo0bGcdNn9w+cbFgkclskaNLJX2hFJz8t16nox+PMvDQpoWKGa/qhcCnmfip8oMtn5XpALlM1I+4snZgVjqM0xz8LAJu0YkgZYHIt5PgA9sK0";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.


        // wait for the start button to be pressed.

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        telemetry.addData("Gyro", "Calibrating");    //
        telemetry.update();
        robot.gyro.calibrate();

        // Send telemetry message to alert driver that we are calibrating;

        while (!isStopRequested() && robot.gyro.isCalibrating()) {
            sleep(50);
            idle();
        }
        telemetry.addData("GyroCalibrated", "GyroCalibrated");
        telemetry.update();
        waitForStart();


        robot.GemGame(this, true);

        robot.gyroDrive(this,0.35,180,14.5,10,0);
        robot.gyroTurn(this,0.4,175,true);
        robot.gyroDrive(this,0.25,90,2,1,175);
        robot.gyroDrive(this,0.25,180,2,1,175);



        robot.releaseLowerCubeArm();

        double rX = 0;
        double rY = 10;
        double rZ = 10;
        double rotation = 0;
        double tZ = -2000;

        relicTrackables.activate();
        double timeStart = getRuntime();
        double gyroTarget;






        RelicRecoveryVuMark vuMark=RelicRecoveryVuMark.from(relicTemplate);
        timeStart = getRuntime();
        double tX=0;


        int vuPosition = 0;

        while (this.opModeIsActive() && this.getRuntime()-timeStart<1 && vuPosition==0 ) {


            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);


                relicTemplate.setLocation(targetOrientation);
                ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
                //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                // location  = listener.getUpdatedRobotLocation();
                if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                    telemetry.addData("VuMark is", "Left");
                    vuPosition = 3;

                } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
                    telemetry.addData("VuMark is", "Right");
                    vuPosition = 1;


                } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
                    telemetry.addData("VuMark is", "Center");
                    vuPosition = 2;

                }



            } else {
                telemetry.addData("VuMark", "not visible");
            }
        }



        double targetTZ = 800;
        if (vuPosition == 3)
            targetTZ = 880;

        else if (vuPosition == 2)
            targetTZ = 680;
        else if (vuPosition == 1)
            targetTZ = 480;
        else targetTZ = 890;


        tZ = -400;

        double gyroTargetAngle = robot.gyroAngle(robot.gyro.getIntegratedZValue());



        while (this.opModeIsActive() && Math.abs(tZ) < targetTZ) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);


                relicTemplate.setLocation(targetOrientation);
                ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
                //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
                // OpenGLMatrix location  = null;

                // location  = listener.getUpdatedRobotLocation();
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));


                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);


                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    //double tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    // rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    // rZ = rot.thirdAngle;

                    rotation = Range.clip(-rY * 0.01, -0.3, 0.3);

                    robot.angleDrive(this, 0.25, 270, rotation);


                    //telemetry.addData("X-Y-Z =", tX + "|" + tY + "|" + tZ);

                    // telemetry.addData("rX-rY-rZ=", rX + "|" + rY + "|" + rZ);
                    telemetry.addData("rY", rY);
                    telemetry.addData("tZ", tZ);

                    telemetry.update();
                }

            } else {
                telemetry.addData("VuMark", "not visible");
            }


        }
        robot.stopRobot();



        gyroTarget = robot.gyroAngle(robot.gyro.getIntegratedZValue()) + rY;

        robot.gyroDrive(this,0.25,0,7,15,gyroTarget);



        robot.ReleaseCube(this);


    }

    String format (OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}



