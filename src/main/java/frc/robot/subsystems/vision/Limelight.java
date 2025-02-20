package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Limelight implements Sendable {
    public static final Distance DEFAULT_HEIGHT_TO_TARGET = Meters.of(0.5);
    private final String name;
    private final VisionTypes.LimelightInfo info;

    private final VisionTypes.CameraDistanceValues cameraDistanceValues;
    public  HashMap<Integer, Double> getTagFacingAngle = new HashMap<>();

    public Limelight(VisionTypes.LimelightInfo info) {
        this.name = LimelightHelpers.sanitizeName(info.name());
        this.info = info;
        this.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
        this.setPipelineIndex(0);   // implicitly sets pipeline type
        this.cameraDistanceValues = new VisionTypes.CameraDistanceValues(
                this.info.heightToCamera(),
                DEFAULT_HEIGHT_TO_TARGET,
                this.info.cameraAngle());

        // TODO: Add robotspace coordinates for camera
        // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#6-special-apriltag-functionality

        updateTagList();
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

    public HashMap<Integer, Double> getTagMap(){
        return getTagFacingAngle;
    }

    private void updateTagList(){
        this.getTagFacingAngle.put(17, 45.0);
        this.getTagFacingAngle.put(18, 0.0);
        this.getTagFacingAngle.put(19, 315.0);
        this.getTagFacingAngle.put(20, 225.0);
        this.getTagFacingAngle.put(21, 180.0);
        this.getTagFacingAngle.put(22, 135.0);

        this.getTagFacingAngle.put(11, 45.0);
        this.getTagFacingAngle.put(10, 0.0);
        this.getTagFacingAngle.put(9, 315.0);
        this.getTagFacingAngle.put(8, 225.0);
        this.getTagFacingAngle.put(7, 180.0);
        this.getTagFacingAngle.put(6, 135.0);

        this.getTagFacingAngle.put(12, 225.0);
        this.getTagFacingAngle.put(13, 135.0);

        this.getTagFacingAngle.put(14, 0.0);
        this.getTagFacingAngle.put(15, 0.0);

        this.getTagFacingAngle.put(16, 270.0);

        this.getTagFacingAngle.put(4, 180.0);
        this.getTagFacingAngle.put(5, 180.0);

        this.getTagFacingAngle.put(3, 90.0);

        this.getTagFacingAngle.put(2, 45.0);
        this.getTagFacingAngle.put(1, 315.0);
    }
}
