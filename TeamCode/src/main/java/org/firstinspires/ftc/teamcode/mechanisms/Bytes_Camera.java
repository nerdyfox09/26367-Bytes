
package org.firstinspires.ftc.teamcode.mechanisms;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BYTES_CONFIG;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Bytes_Camera {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(CameraName cameraName, HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(BYTES_CONFIG.MODE_DEBUG)
                .setDrawTagOutline(BYTES_CONFIG.MODE_DEBUG)
                .setDrawAxes(BYTES_CONFIG.MODE_DEBUG)
                .setDrawCubeProjection(BYTES_CONFIG.MODE_DEBUG)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setSuppressCalibrationWarnings(false)
                .setLensIntrinsics(BYTES_CONFIG.HW_SENSORS_CAMERA_FX,
                        BYTES_CONFIG.HW_SENSORS_CAMERA_FY,
                        BYTES_CONFIG.HW_SENSORS_CAMERA_CX,
                        BYTES_CONFIG.HW_SENSORS_CAMERA_CY)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(cameraName);
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId){
        if(detectedId == null){
            return;
        }
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId (int id) {
        telemetry.addData("looking for id: ", id);
        telemetry.addData("detected tags count: ", detectedTags.size());
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
        }
    }
}
