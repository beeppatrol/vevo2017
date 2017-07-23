package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/*

    Class to track for objects using Vuforia
    Created by Sean 7/23/2017

    For details about this class, see
    https://www.youtube.com/watch?v=2z-o9Ts8XoE
    see 15:40 for demo of code working below

    TODO: add methods/class to navigate to the tracked object

 */
public class VuforiaOpImpl extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // back or front camera
        params.vuforiaLicenseKey = getLicenseKey();
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // also TEAPOT, none (we can experiment with these).
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, NUMBER_OF_BEACONS);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset(FILE_NAME);
        beacons.setName("Wheels"); // name of image from xml file
        beacons.setName("Tools");
        beacons.setName("Lego");
        beacons.setName("gears");

        waitForStart();
        beacons.activate();  // vuforia starts tracking for objects (no point calling this until after waitForStart() )

        while(opModeIsActive()) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                if (null!=pose) { // may be null if camera doesn't find target object
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Translation: ", translation);
                    // Math.atan2(x,y) where x,y are the coordinates coming back from the camera/phone
                    // phone may be in landscape or portrait position, so change accordingly
                    // - landscape: translation.get(0), translation.get(2)
                    // - portrait: translation.get(1), translation.get(2)
                    double degreestoTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beacon.getName() + "-Degrees: ", degreestoTurn);
                }
            }
            telemetry.update();
        }

    }

    private static String getLicenseKey () {
        // url: developer.vuforia.com
        // user/pw: beeppatrol/Team96snore
        return LICENSE_KEY;
    }

    private static final String LICENSE_KEY = "AehWUEP/////AAAAGdLM1Ir3CEUunWFOGlSVegZ02oYjauBrfpYGcP/MNvZGEWO15KaOdjuIx0XAGISDJtiT9pfALwG5bGHfY2d5LVLV3jBq+2vLfcYh7zxUbHOcJpPfbzpUDVkGI5WHZlZ6IaqoCAEPznkxcZ5uyMwfZr1qyZp9LVTTAFhYwjRgSuF4/mcjzI3/ujUOZEKUzIOQbSlAPyNkiNMnRA0RHlzK7djpkXvghYsX7LYJDnJc5Fvpi6mqZqI+lyco0jnUHhMh4l7HczZ1HbKTAwuJFqc3aQab8bnjw9QegJb62vURA/ljwEIEUhT6mEGx+XJSOUA+KCwi/WDnKcZwOZr43VqmHPgLCvJmTFpVeOdBY4ozX5/J";
    private static final int  NUMBER_OF_BEACONS = 4; // number of beacons on the FTC game floor
    private static final String FILE_NAME = "FTC_2016-17";  // name of xml file which contains the name of the images (see the project's FtcRobotController/assets folder)


    // find an absolute position on the field
    // see ftc/doc/tutorial folder in this project, file: FTC_FieldCoordinateSystemDefinition.pdf
    // for information about how the field coordinates work..
    //
    // https://www.youtube.com/watch?v=2z-o9Ts8XoE
    // see @16:00 for brief demo
    // see ConceptVuforiaNavigation.java class

}
