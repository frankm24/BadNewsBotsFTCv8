package badnewsbots;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class InterOpStorage {
    public enum Alliance {
        BLUE,
        RED,
        NONE
    }
    public static Pose2d currentPose = new Pose2d();
    public static Alliance alliance = Alliance.NONE;
    private static final double tileSize = 23.5;
    public static List<Junction> junctions = Arrays.asList(
            new Junction(-tileSize, 0, Junction.JunctionType.HIGH),
            new Junction(0, tileSize, Junction.JunctionType.HIGH),
            new Junction(tileSize, 0, Junction.JunctionType.HIGH),
            new Junction(0, -tileSize, Junction.JunctionType.HIGH),

            new Junction(-tileSize, tileSize, Junction.JunctionType.MIDDLE),
            new Junction(tileSize, tileSize, Junction.JunctionType.MIDDLE),
            new Junction(tileSize, -tileSize, Junction.JunctionType.MIDDLE),
            new Junction(-tileSize, -tileSize, Junction.JunctionType.MIDDLE),

            new Junction(-2*tileSize, tileSize, Junction.JunctionType.LOW),
            new Junction(-tileSize, 2*tileSize, Junction.JunctionType.LOW),
            new Junction(tileSize, 2*tileSize, Junction.JunctionType.LOW),
            new Junction(2*tileSize, tileSize, Junction.JunctionType.LOW),
            new Junction(2*tileSize, -tileSize, Junction.JunctionType.LOW),
            new Junction(tileSize, -2*tileSize, Junction.JunctionType.LOW),
            new Junction(-tileSize, -2*tileSize, Junction.JunctionType.LOW),
            new Junction(-2*tileSize, -tileSize, Junction.JunctionType.LOW)
    );
}
