package org.frc6423.lib.util;

import org.wpilib.math.geometry.Pose2d;

public class NodeSelector {
    public static Pose2d selectClosestNode(Pose2d[] nodes, Pose2d robotPose, double threshold) {
        double minDistance = Double.MAX_VALUE;
        int closestNodeIndex = -1;

        for (int i = 0; i < nodes.length; i++) {
            double distance = robotPose.getTranslation().getDistance(nodes[i].getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestNodeIndex = i;
            }
        }

        if (minDistance <= threshold) {
            return nodes[closestNodeIndex];
        }
        return null;
    
    }
}
