package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Meters;

public class Limelight implements Sendable {
    public static final Distance DEFAULT_HEIGHT_TO_TARGET = Meters.of(0.5);
    private final String name;
    private final VisionTypes.LimelightInfo info;

    private final VisionTypes.CameraDistanceValues cameraDistanceValues;
    private final LimelightPoseEstimator poseEstimator;

    public Limelight(VisionTypes.LimelightInfo info) {
        this.name = LimelightHelpers.sanitizeName(info.name());
        this.info = info;
        this.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
        this.setPipelineIndex(0);   // implicitly sets pipeline type
        this.cameraDistanceValues = new VisionTypes.CameraDistanceValues(
                this.info.heightToCamera(),
                DEFAULT_HEIGHT_TO_TARGET,
                this.info.cameraAngle());
        this.poseEstimator = new LimelightPoseEstimator(this.name, true);

        // TODO: Add robotspace coordinates for camera
        // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#6-special-apriltag-functionality
    }

    public LimelightPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public VisionTypes.LimelightInfo getInfo() {
        return this.info;
    }

    public String getName() {
        return this.name;
    }


    public void setPipeline(Pipeline pipeline) {
        this.setPipelineIndex(pipeline.pipelineNumber);
    }

    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(this.name, pipelineIndex);
    }

    public int getPipelineIndex() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(this.name);
    }

    public Optional<Pipeline> getPipeline() {
        return Pipeline.fromPipelineIndex(this.getPipelineIndex());
    }

    public double getAprilTagID() {
        return LimelightHelpers.getFiducialID(this.name);
    }

    public String getNeuralClassID() {
        return LimelightHelpers.getNeuralClassID(this.name);
    }

    public Command requestPipeline(int pipelineNumber) {
        return Commands.runOnce(() -> this.setPipelineIndex(pipelineNumber));
    }

    public VisionTypes.TargetInfo getTargetInfo() {
        return new VisionTypes.TargetInfo(
                LimelightHelpers.getTV(this.name),
                LimelightHelpers.getTX(this.name),
                LimelightHelpers.getTY(this.name),
                LimelightHelpers.getTA(this.name),
                (int) getAprilTagID()
        );
    }

    public VisionTypes.PoseInfo getBotPoseForCurrentAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Pose2d pose = DriverStation.Alliance.Red.equals(alliance.get()) ?
                    LimelightHelpers.getBotPose2d_wpiRed(this.name) :
                    LimelightHelpers.getBotPose2d_wpiBlue(this.name);
            double latency = LimelightHelpers.getLatency_Capture(this.name) +
                    LimelightHelpers.getLatency_Pipeline(this.name);
            return new VisionTypes.PoseInfo(pose, latency);
        } else {
            DataLogManager.log("[Limelight] No alliance information available from DriverStation");
            throw new RuntimeException("No alliance information available");
        }
    }


    public void setLEDMode(VisionTypes.LEDMode ledMode) {
        Consumer<String> selector = switch (ledMode) {
            case CurrentPipeline -> LimelightHelpers::setLEDMode_PipelineControl;
            case ForceOff -> LimelightHelpers::setLEDMode_ForceOff;
            case ForceBlink -> LimelightHelpers::setLEDMode_ForceBlink;
            case ForceOn -> LimelightHelpers::setLEDMode_ForceOn;
        };
        selector.accept(this.name);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Limelight");
        builder.addStringProperty("Name", this::getName, null);
        builder.addDoubleProperty("AprilTagID", this::getAprilTagID, null);
        builder.addDoubleProperty("CurrentPipeline", ()-> LimelightHelpers.getCurrentPipelineIndex(this.name),(idx) -> LimelightHelpers.setPipelineIndex(this.name, (int)idx));

        VisionTypes.TargetInfo targetInfo = this.getTargetInfo();
        builder.addBooleanProperty("TargetInfo/hasTarget", targetInfo::hasTarget, null);
        builder.addDoubleProperty("TargetInfo/xOffset", targetInfo::xOffset, null);
        builder.addDoubleProperty("TargetInfo/yOffset", targetInfo::yOffset, null);
        builder.addDoubleProperty("TargetInfo/targetArea", targetInfo::targetArea, null);
    }
}
