package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.w8wjb.ftc.AdafruitNeoDriver;

import android.graphics.Color;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "MecanumWorld", group = "TeleOp")

public class MecanumLinear extends LinearOpMode {


    //DriveTrain Motors
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone, HangR, HangL, Grabber;
    private CRServo IntakeRoller, GrabRoller;







    //limitswitch
    private TouchSensor LimitSwitch;
    //BoxColorSensor
    private NormalizedColorSensor Color, ColorFront;

    private ColorSensor ColorSense, ColorFrontSense;

    long startTime = System.currentTimeMillis();



    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;

    int Pixels = 0;

    public static final int NUM_PIXELS = 60;

    public final int thirdPixels = 15;

    public final int twoThirdPixels = 35;



    AdafruitNeoDriver neopixels;

       float[] stripPurple =  new float[] {278 , 89, 66 };

       float[] stripGreen = new float[] {101 , 42, 66 };

       float[] stripYellow = new float[] {20, 100, 100};

       float[] stripWhite = new float[] {0 , 0, 100};







    boolean Rumbled = false;

    boolean yPressed = false;

    boolean boardAdjust = false;



    boolean pixelBack = false;


    boolean currentPress = false;
    boolean pastPress = false;

    boolean dropPress = true;
    //Motor Power


    double drive;
    double turn;

    long pixelTime;
    double strafe;
    double SlidePower;
    double fLeftPow, fRightPow, bLeftPow, bRightPow;

    boolean gameOn = true;


    boolean IntakeReady = false;



    @Override
    public void runOpMode() throws InterruptedException {



        if (opModeInInit()) {
            initCode();
         //   BucketL.setPosition(0.5);
         //   BucketR.setPosition(0.5 );
        }

        waitForStart();

        while(opModeIsActive()) {

            if(IntakeReady){
                OpenBox();
            }



            if (LimitSwitch.isPressed()) {
                telemetry.addData("Intake", "Ready");
                if (!Rumbled) {
                    gamepad2.rumble(500);
                    gamepad2.rumble(500);
                    Rumbled = true;

                }
            } else {
                Rumbled = false;
                gamepad2.stopRumble();
                telemetry.addData("Intake", " Not Ready");
            }



            telemetry.addData("ServoR", HangR.getPosition());
            telemetry.addData("ServoL", HangL.getPosition());

            telemetry.addData("SlideL:", SlideL.getCurrentPosition());
            if (Color instanceof DistanceSensor) {
                telemetry.addData("Back (cm)", "%.3f", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
                telemetry.addData("Front (cm)", "%.3f", ((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM));
            }
            telemetry.addData("Pixels:", Pixels);


            //Movement Controller


            right_drivePower = gamepad1.right_stick_y *-1;
            back_left_drivePower = gamepad1.left_stick_y * -1;
            left_drivePower = gamepad1.left_stick_y * -1;
            back_right_drivePower = gamepad1.right_stick_y * -1;


            left_drive.setPower(left_drivePower);
            right_drive.setPower(right_drivePower);
            back_left_drive.setPower(left_drivePower);
            back_right_drive.setPower(right_drivePower);


            boolean rightbumper = gamepad1.right_bumper; //Strafe Right
            boolean leftbumper = gamepad1.left_bumper; //Strafe Left


            boolean UpSlideBumper = gamepad2.right_bumper;
            boolean DownSlideBumper = gamepad2.left_bumper;

            long endTime = System.currentTimeMillis() - startTime;

            //attachments


            if (rightbumper) {

                left_drive.setPower(1); // left drive is 0
                right_drive.setPower(-1); // right drive is 2
                back_left_drive.setPower(-1); // back left drive is 1
                back_right_drive.setPower(1); // back right drive is 3


            } else if (leftbumper) {

                left_drive.setPower(-1);
                right_drive.setPower(1);
                back_left_drive.setPower(1);
                back_right_drive.setPower(-1);


            }
            telemetry.addData("Slides", SlideL.getCurrentPosition());
            telemetry.update();




            //Fwd Bckwd
            if(gamepad1.right_trigger > 0.3){
                left_drive.setPower(1);
                right_drive.setPower(1);
                back_left_drive.setPower(1);
                back_right_drive.setPower(1);
            }

            if(gamepad1.left_trigger > 0.3){
                left_drive.setPower(-1);
                right_drive.setPower(-1);
                back_left_drive.setPower(-1);
                back_right_drive.setPower(-1);
            }
            //Auto Forward and Backward

       /* if(gamepad1.right_trigger > 0.3){
            left_drive.setPower(1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(1);
        }
        else if(gamepad1.left_trigger > 0.3){
            left_drive.setPower(-1);
            right_drive.setPower(-1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(-1);



        }
        else{

        } */


            //Led code


            if (Pixels == 0 /*&& endTime < 85000*/) {
                  showRed();
            }

            if (Pixels == 1 /*&& endTime < 85000*/) {
               // colorsDefined(stripWhite , getBackColor());
                 showGreen();
            }

            if (Pixels == 2 /*&& endTime < 85000*/) {
               showPurple();


                //colorsDefined(getFrontColor(), getBackColor());
            }

            if (endTime > 80000 && Pixels == 0 || endTime > 80000 && Pixels == 1 || endTime > 80000 && Pixels == 2) {


                if (endTime > 80100 && endTime < 80500) {
                    showGold();
                }
                if (endTime > 80500 && endTime < 81000) {
                    showPurple();
                }
                if (endTime > 81500 && endTime < 82000) {
                    showGold();

                }
                if (endTime > 82000 && endTime < 82500) {
                    showPurple();
                }
                if (endTime > 82500 && endTime < 83000) {
                    showGold();
                }
                if (endTime > 83500 && endTime < 84000) {
                    showPurple();
                }
                if (endTime > 84000 && endTime < 84500) {
                    showGold();

                }
                if (endTime > 85000) {
                    showPurple();
                }


            }

            // CLAW ROTATION


            //Slide Goes Down
            if (!LimitSwitch.isPressed()) {


                if (DownSlideBumper && !yPressed) {


                    IntakeBox();
                    boardAdjust = false;

                    SlideL.setPower(-.6);
                    SlideR.setPower(.6);



                } else if (DownSlideBumper) {
                    SlideL.setPower(-.6);
                    SlideR.setPower(.6);
                }


            } else {
                telemetry.addData("Hold Position: ", BucketHold.getPosition());
                telemetry.update();
                StopSlides();
                OpenBox();
                IntakeReady = true;


                if (Pixels == 2) {
                    CloseBox();
                }

                //if 0 pixels
                else if (Pixels < 2 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) && !pixelBack) {
                    Pixels++;
                    pixelBack = true;
                }
                else if(Pixels < 2 && (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5) && pixelBack){
                    Pixels++;
                }


            }

            if (UpSlideBumper && !yPressed && !boardAdjust) {
                IntakeReady = false;
                CloseBox();
                IntakeBox();
                SlideL.setPower(.75);
                SlideR.setPower(-.75);


            } else if (UpSlideBumper) {
                IntakeReady = false;
                SlideL.setPower(.75);
                SlideR.setPower(-.75);
            } else if (!DownSlideBumper && !LimitSwitch.isPressed()) {

                HoldSlides();


            }


            //Drop Box on Board
            if (gamepad2.y) {
                yPressed = true;
                BoardDropBox();

            }


            // open
            if (gamepad2.left_trigger > 0.3 && yPressed) {
                OpenBox();

                boardAdjust = true;
                Pixels = 0;
                yPressed = false;
                pixelBack = false;

            }
           else if(gamepad2.left_trigger > 0.3){
                Pixels = 0;
                OpenBox();
                pixelBack = false;
            }




            if (gamepad2.right_trigger > 0.3 && yPressed) {
                dropBox();

                boardAdjust = true;
                Pixels = 0;
                yPressed = false;
                pixelBack = false;



            } else if (gamepad2.right_trigger > 0.3) {
                Pixels = 0;
                dropBox();
                pixelBack = false;


            }

            /*
             pastPress = currentPress;
            currentPress = gamepad2.right_trigger > 3;
            if(currentPress && !pastPress) {
                dropPress = !dropPress;
                if (gamepad2.right_trigger > 0.3 && yPressed && dropPress) {
                    dropBox();

                    boardAdjust = true;
                    Pixels = 0;
                    yPressed = false;
                    pixelBack = false;



                } else if (gamepad2.right_trigger > 0.3 && dropPress) {
                    dropBox();
                    pixelBack = false;


                }
            }*/
            if (gamepad2.x) {
                GrabRoller.setPower(0.85);
                useGrabber();
                //Intake.setPower(1); // was 0.9 until 3/1
                //IntakeRoller.setPower(-0.8);
            }
            else {
                setGrabber();
                GrabRoller.setPower(0);
            }


            //intake
            if (gamepad2.a && IntakeReady) {
                Intake.setPower(1); // was 0.9 until 3/1
                IntakeRoller.setPower(0.8);

            }
            // outtake
            else if (gamepad2.b) {
                Intake.setPower(-0.6);
                IntakeRoller.setPower(-0.8);
            } else {
                Intake.setPower(0);
                IntakeRoller.setPower(0);
            }

            //drone launching and resetting
            if (gamepad2.back) {
                Drone.setPosition(0.75);
            } else {
                Drone.setPosition(0.55);

            }



            // Hanging / LeadScrew Binds

            if (gamepad2.dpad_up) {
                HangR.setPosition(0.5); // correct
                HangL.setPosition(0.1);

            }
            if (gamepad2.dpad_down) {
                HangR.setPosition(0);
                HangL.setPosition(0.6); //correct

            }
            if(isStopRequested()){
                left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                left_drive.setPower(0);
                right_drive.setPower(0);
                back_left_drive.setPower(0);
                back_right_drive.setPower(0);
            }
        }


            left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            left_drive.setPower(0);
            right_drive.setPower(0);
            back_left_drive.setPower(0);
            back_right_drive.setPower(0);
            colorOff();

    }




    private void CloseBox(){
        //BucketHold.setPosition(0); //close
        BucketHold.setPosition(0.7);
    }
    private void OpenBox(){
      //  BucketHold.setPosition(0.7); //close
        BucketHold.setPosition(0.5);

    }
    private void dropBox(){
        BucketHold.setPosition(0.57);

        sleep(100);
        BucketHold.setPosition(0.7);
        sleep(750);
    }


    private void useGrabber(){
        Grabber.setPosition(0.82);
    }

    private void setGrabber(){
        Grabber.setPosition(0.47);
    }

    private void BoardDropBox(){
        //BucketL.setPosition(0.02);
        BucketR.setPosition(0.72);

    }
    private void HoldSlides(){
        SlideL.setPower(0.08);
        SlideR.setPower(-0.08);
    }
    private void StopSlides(){

        SlideL.setPower(0);
        SlideR.setPower(0);
    }
    private void IntakeBox(){
       BucketL.setPosition(0.3);
       BucketR.setPosition(0.34);

    }

    private void showPurple(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {278 , 89, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGreen(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {101 , 42, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGold(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {247 , 98, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    private void showRed(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {6 , 100, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }


    private void colorOff(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {0 , 0, 0 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    private float[] getBackColor(){
     /*   //Yellow
        if ((1000 < ColorSense.red() && ColorSense.red() < 1200) && (400 < ColorSense.blue()) && (ColorSense.blue() < 550) && (1500 < ColorSense.green()) && (ColorSense.green() < 1700)) {
            return stripYellow;
        }

        //White
        if ((2000 < ColorSense.red() && ColorSense.red() < 2250) && (3600 < ColorSense.blue()) && (ColorSense.blue() < 3750) && (3850 < ColorSense.green()) && (ColorSense.green() < 4000)) {


            return stripWhite;
        }

        //Green
        if ((300 < ColorSense.red() && ColorSense.red() < 400) && (400 < ColorSense.blue()) && (ColorSense.blue() < 550) && (900 < ColorSense.green()) && (ColorSense.green() < 1100)) {
            return stripGreen;
        }

        //Purple
        if ((700 < ColorSense.red() && ColorSense.red() < 950) && (1850 < ColorSense.blue()) && (ColorSense.blue() < 1990) && (1200 < ColorSense.green()) && (ColorSense.green() < 1400)) {
            return stripPurple;
        }

        return stripWhite;


      */

        telemetry.addData("Normalized Color Front: " , ColorFront.getNormalizedColors().toColor());
        telemetry.addData("Normalized Color Back: ", Color.getNormalizedColors().toColor());

        telemetry.update();
        //Purple

        if(Color.getNormalizedColors().toColor() < -120000000  && Color.getNormalizedColors().toColor() > -140000000){
            return stripPurple;
        }

        //Yellow
        if(Color.getNormalizedColors().toColor() < -220000000  && Color.getNormalizedColors().toColor() > -250000000){
            return stripYellow;
        }

        //White
        if(Color.getNormalizedColors().toColor() < -48000000 && Color.getNormalizedColors().toColor() > -51000000 ){
            return stripWhite;
        }

        return stripGreen;
        
    }

    private float[] getFrontColor() {
       /* //Yellow
        if ((100 < ColorFrontSense.red() && ColorFrontSense.red() < 200) && (100 < ColorFrontSense.blue()) && (ColorFrontSense.blue() < 200) && (200 < ColorFrontSense.green()) && (ColorFrontSense.green() < 300)) {
            return stripYellow;
        }
            //White
            if ((240 < ColorFrontSense.red() && ColorFrontSense.red() < 350) && (400 < ColorFrontSense.blue()) && (ColorFrontSense.blue() < 500) && (450 < ColorFrontSense.green()) && (ColorFrontSense.green() < 550)) {


                return stripWhite;
            }
            //Green
            if ((50 < ColorFrontSense.red() && ColorFrontSense.red() < 150) && (100 < ColorFrontSense.blue()) && (ColorFrontSense.blue() < 200) && (180 < ColorFrontSense.green()) && (ColorFrontSense.green() < 280)) {
                return stripGreen;
            }
            //Purple
            if ((100 < ColorFrontSense.red() && ColorFrontSense.red() < 200) && (250 < ColorFrontSense.blue()) && (ColorFrontSense.blue() < 380) && (200 < ColorFrontSense.green()) && (ColorFrontSense.green() < 300)) {
                return stripPurple;
            }
            */


        telemetry.addData("Normalized Color Front: " , ColorFront.getNormalizedColors().toColor());
        telemetry.addData("Normalized Color Back: ", Color.getNormalizedColors().toColor());

        telemetry.update();
        //Purple

        if(Color.getNormalizedColors().toColor()  < -1400000000 && Color.getNormalizedColors().toColor() > -1700000000 ){
            return stripPurple;
        }
        //White

        if(Color.getNormalizedColors().toColor() < -490000000  && Color.getNormalizedColors().toColor() > - 510000000  ){
            return stripWhite;
        }
        //Yellow
        if(Color.getNormalizedColors().toColor() < -2000000000 && Color.getNormalizedColors().toColor() >-220000000 ){
            return stripYellow;
        }

            return stripGreen;
        }




    private void colorsDefined(float[] frontColor, float[] BackColor){

        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < thirdPixels; i++) {
            int color = android.graphics.Color.HSVToColor(frontColor);
            colors[i] = color;
        }


        for (int i= thirdPixels;  i < twoThirdPixels ; i++) {
            int color = android.graphics.Color.HSVToColor(BackColor);
            colors[i] = color;
        }

        for (int i=twoThirdPixels; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(frontColor);
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    private void initCode(){
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");

        //EXPANSION HUB MOTORS
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");
        Intake = hardwareMap.dcMotor.get("Intake");


        //MOTOR DIRECTION SWITCHING
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);

        //SENSORS
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Color = hardwareMap.get(NormalizedColorSensor.class,"Color");
        ColorFront = hardwareMap.get(NormalizedColorSensor.class,"Color2");

        ColorSense = hardwareMap.get(ColorSensor.class, "Color");
        ColorFrontSense = hardwareMap.get(ColorSensor.class,"Color2");


        //SERVOS
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Drone = hardwareMap.get(Servo.class, "Drone");

        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");
        GrabRoller = hardwareMap.get(CRServo.class, "GrabRoller");




         Grabber = hardwareMap.get(Servo.class, "Grab");


        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");







        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neopixels");

        neopixels.setNumberOfPixels(NUM_PIXELS);

        //zero Power


        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      //  GrabRoller.setPower(0);
        HangR.setPosition(0.5); // correct
        HangL.setPosition(0.1);
        // HangR.setPosition(0.5);
        //  HangL.setPosition(0.5);
        //O position for Servos Default
/*
        BucketR.setPosition(0);
        BucketL.setPosition(1);
*/



    }

}