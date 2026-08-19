package org.firstinspires.ftc.teamcode.pedroPathing;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "camera")
public class opencv extends LinearOpMode {
    double cx = 0;
    double cy = 0;
    double width = 0;

    private OpenCvCamera limelight;
    private static final int CAMERA_WIDTH = 11; // width of the limelight
    private static final int CAMERA_HEIGHT = 11; // height

    //calcular distância

    public static final double ObjectWidthInRealWorldUnits = 3.74; //
    public static final double FocalLenght = 728;// focal leght of the camera in pixels

    @Override
    public void runOpMode() throws InterruptedException {

        initOpenCV();

    }


}
