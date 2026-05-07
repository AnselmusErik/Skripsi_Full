package routing;

import core.Connection;
import core.DTNHost;
import core.Settings;
import core.SimClock;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class LinkDurationRouter extends ActiveRouter {

    /* ================== CONFIG ================== */
    protected double durationThreshold = 20; // bisa kamu ubah

    /* ================== TRACKING ================== */
    private Map<DTNHost, double[]> lastPositions = new HashMap<>();
    private Map<DTNHost, Double> lastTimes = new HashMap<>();

    /* ================== CONSTRUCTOR ================== */
    public LinkDurationRouter(Settings s) {
        super(s);
    }

    public LinkDurationRouter(LinkDurationRouter r) {
        super(r);
    }

    /* ================== VELOCITY ================== */
    protected double[] getVelocity(DTNHost host) {
        double currentX = host.getLocation().getX();
        double currentY = host.getLocation().getY();
        double currentTime = SimClock.getTime();

        double vx = 0, vy = 0;

        if (lastPositions.containsKey(host)) {
            double[] lastPos = lastPositions.get(host);
            double lastTime = lastTimes.get(host);

            double dt = currentTime - lastTime;
            if (dt > 0) {
                vx = (currentX - lastPos[0]) / dt;
                vy = (currentY - lastPos[1]) / dt;
            }
        }

        lastPositions.put(host, new double[]{currentX, currentY});
        lastTimes.put(host, currentTime);

        return new double[]{vx, vy};
    }

    /* ================== LINK DURATION ================== */
    protected double estimateLinkDuration(DTNHost a, DTNHost b) {

        double[] va = getVelocity(a);
        double[] vb = getVelocity(b);

        // ================== SPEED ==================
        double vi = Math.sqrt(va[0]*va[0] + va[1]*va[1]);
        double vj = Math.sqrt(vb[0]*vb[0] + vb[1]*vb[1]);

        // ================== COS ALPHA ==================
        double dot = (va[0]*vb[0] + va[1]*vb[1]);
        double cosAlpha = 0;

        if (vi != 0 && vj != 0) {
            cosAlpha = dot / (vi * vj);
        }

        // ================== RELATIVE SPEED ==================
        double Vij = Math.sqrt(
                vi*vi + vj*vj - 2 * vi * vj * cosAlpha
        );

        if (Vij == 0) {
            return Double.MAX_VALUE; // tidak akan putus
        }

        // ================== DISTANCE ==================
        double dx = a.getLocation().getX() - b.getLocation().getX();
        double dy = a.getLocation().getY() - b.getLocation().getY();
        double d = Math.sqrt(dx*dx + dy*dy);

        double r = getHost().getInterfaces().get(0).getTransmitRange();

        // ================== COS BETA ==================
        double cosBeta = d / (2 * r);

        if (cosBeta > 1) {
            return 0; // sudah di luar jangkauan
        }

        // ================== ACTIVE DISTANCE ==================
        double Dij = 2 * r * cosBeta;

        // ================== LINK DURATION ==================
        double Tij = Dij / Vij;

        return Tij;
    }

    /* ================== FILTER CONNECTION ================== */
    protected List<Connection> filterConnections(List<Connection> connections) {

        List<Connection> filtered = new ArrayList<>();

        for (Connection con : connections) {
            DTNHost other = con.getOtherNode(getHost());

            double duration = estimateLinkDuration(getHost(), other);

            if (duration >= durationThreshold) {
                filtered.add(con);
            }
        }

        return filtered;
    }
}