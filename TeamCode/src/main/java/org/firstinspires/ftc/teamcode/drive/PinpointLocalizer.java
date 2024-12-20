package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.myConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class PinpointLocalizer implements Localizer {

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;
    GoBildaPinpointDriver odo;
    public PinpointLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {


        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;
        //HI

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(myConstants.X_OFFSET, myConstants.Y_OFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(myConstants.xEncoder_DIRECTION, myConstants.yEncoder_DIRECTION);
        odo.resetPosAndIMU();
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)


    }


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2D pose = odo.getPosition();
        return new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        //Pose2D pose = new Pose2D(DistanceUnit.INCH, pose2d.getX(), pose2d.getY(), AngleUnit.RADIANS, pose2d.getHeading());
        //odo.setPosition(pose);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2D pose = odo.getVelocity();
        return new Pose2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void update() {
        odo.update();
    }

    //@Override
    //public
}
