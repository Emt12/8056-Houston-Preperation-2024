package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.CustomEncoder;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
    private static final PhotonCamera camera = new PhotonCamera("targetcam");
    
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(-180, 0, 0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

    public Vision() {
        camera.setPipelineIndex(0);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public Pose3d robotPose3d() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            //List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();
            int targetID = target.getFiducialId();    

            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    aprilTagFieldLayout.getTagPose(targetID).get(),
                    robotToCam);

            return robotPose;
        }

        return new Pose3d();
    }
    

    public PhotonTrackedTarget getTarget(){
         var result = camera.getLatestResult();
        var resList = result.getTargets();

        boolean hasTargets = result.hasTargets();
        PhotonTrackedTarget target = null;
        if(hasTargets){
         for(int i = 0; i < resList.size(); i++){
                if(resList.get(i).getFiducialId() == 8)//add
                {
                    target = resList.get(i);
                }
            }
        }
        return target;
    }
    /**
     * @return 0. yaw, 1. pitch, 2. area, 3. skew 4. range
     * if range < 0 or any of the values is negative, function failed to find target
     */
    public List<Double> getData(PhotonTrackedTarget target){
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                0.19,
                1.36,
                Units.degreesToRadians(25),
                Units.degreesToRadians(pitch)
            );

        List<Double> list = List.of(yaw, pitch, area, skew, range);
        return list;
    }
    
    private static double getCameraWristRatio(double cameraMinAngle, double cameraMaxAngle, double wristMinAngle, double wristMaxAngle){
        return (cameraMaxAngle-cameraMinAngle) / (wristMaxAngle-wristMinAngle);
    }

    private double convertForWrist(double desiredAngle, double camWristRatio){
        return desiredAngle * camWristRatio;
    }

    public double calculateWristTick(){
        double wristMaxTick = 0;
        double wristMinTick = 2000;

        double wristStartAngle = 45;
        double wristEndAngle = 25;

        double cameraMinAngle = 20;
        double cameraMaxAngle = 80;

        double camWristRatio = getCameraWristRatio(cameraMinAngle, cameraMaxAngle, cameraMinAngle, cameraMaxAngle);
        double wristAngle = convertForWrist(getAngle(), camWristRatio);


        double tickAngleRatio = CustomEncoder.mapTickToAngle(wristMinTick, wristMaxTick, wristStartAngle, wristEndAngle, true);

        double converted = CustomEncoder.convertAngleToTick(tickAngleRatio, wristAngle);

        return converted;
    }

    public double getAngle(){
        List<Double> list = getData(getTarget());
        return list.get(1); // 320, 40 veya -40 ---- 86 derece
    }

    @Override
    public void periodic() {
        PhotonTrackedTarget target = getTarget();
        double distance = -999999.0;
        boolean shoot = false;
        double angle = -999999.0;
        if(target != null){
            List<Double> list = getData(target);
            distance = list.get(4);
            angle = list.get(1);
            shoot = false;
            if (distance >= 1.9 && distance <= 2.25){
                shoot = true;
            }
            else{
                shoot = false;
            }

        }
        SmartDashboard.putBoolean("Shoot", shoot);
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angle for it", angle);
        super.periodic();
    }

}
