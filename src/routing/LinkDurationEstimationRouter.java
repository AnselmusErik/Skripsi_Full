package routing;

import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Message;
import core.Settings;
import core.SimClock;

import java.util.HashMap;
import java.util.Map;

/**
 * Link Duration Estimation Router for THE ONE Simulator.
 *
 * Estimates link duration between two nodes based on their current positions
 * and destination positions (from RandomWaypoint mobility model), using the
 * analytical model from:
 *
 *   Wu et al., "Impact of Node Mobility on Link Duration in Multihop Mobile
 *   Networks," IEEE Transactions on Vehicular Technology, vol. 58, no. 5,
 *   June 2009.
 *
 * Estimation pipeline:
 *   1. Compute velocity vectors V1 and V2 from current → destination positions.
 *   2. Compute relative velocity V12 = V2 - V1.
 *   3. Compute angle alpha between V1 and V2 via the dot product formula:
 *          cos(alpha) = (V1 · V2) / (||V1|| * ||V2||)
 *   4. Compute relative speed Vij using the law of cosines:
 *          Vij = sqrt(Vi^2 + Vj^2 - 2*Vi*Vj*cos(alpha))
 *   5. Compute active distance Dij = 2r * cos(beta),
 *      where beta is the incident angle of the relative velocity to the
 *      observer node's transmission range. Beta is derived from the
 *      relative position of the two nodes:
 *          beta = arcsin( perpendicular_distance / r )
 *      and clamped to [0, pi/2].
 *   6. Estimate link duration:
 *          Tij = Dij / Vij
 */
public class LinkDurationEstimationRouter implements RoutingDecisionEngine {

    public static final String LDER_NS = "LDER";

    /** Settings key for detection/transmission range (metres). */
    public static final String DETECTION_RANGE_SETTING = "detectionRange";

    /** Default detection range in metres if not specified in settings. */
    private static final double DEFAULT_DETECTION_RANGE = 10.0;

    /** Transmission / detection range of this node. */
    private double detectionRange;

    private double durationThreshold;

//    private int[] minAndMaxSpeed;

    /**
     * FIX: Cache speed terakhir yang valid per-host.
     * Saat node pause, getPath() == null sehingga speed terbaca 0.0.
     * Cache ini di-update tiap tick via update() selama path aktif.
     */
    private final Map<DTNHost, Double> lastKnownSpeed = new HashMap<>();

    /**
     * FIX: Cache direction unit vector terakhir per-host.
     * Saat node pause, getNextWaypoint() mengembalikan posisi saat ini
     * sehingga displacement = (0,0) → dirX = dirY = 0.
     * Akibatnya velocity = speed * (0,0) = 0 meskipun speed sudah benar.
     * Cache ini menyimpan direction terakhir yang valid agar tidak hilang
     * saat node pause.
     * Disimpan sebagai double[] { dirX, dirY }.
     */
    private final Map<DTNHost, double[]> lastKnownDirection = new HashMap<>();

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------

    public LinkDurationEstimationRouter(Settings _settings) {
        Settings s = new Settings(LDER_NS);

//        this.minAndMaxSpeed = s.getCsvInts("speed", 2);

//        this.detectionRange = s.getDouble(DETECTION_RANGE_SETTING, DEFAULT_DETECTION_RANGE);
        this.detectionRange = (new Settings("btInterface")).getDouble("transmitRange", DEFAULT_DETECTION_RANGE);
        this.durationThreshold = s.getDouble("durationThreshold", 0);
    }

    public LinkDurationEstimationRouter(LinkDurationEstimationRouter proto) {
//        this.minAndMaxSpeed = proto.minAndMaxSpeed;
        this.detectionRange = proto.detectionRange;
        // Tidak copy cache — setiap node punya instance router sendiri.
        // Map akan diisi ulang oleh update() sejak awal simulasi.
    }

    // -----------------------------------------------------------------------
    // RoutingDecisionEngine interface
    // -----------------------------------------------------------------------

    @Override
    public void connectionUp(DTNHost thisHost, DTNHost peer) {
        // No action required on connection establishment.
    }

    @Override
    public void connectionDown(DTNHost thisHost, DTNHost peer) {
        // No action required on connection teardown.
    }

    @Override
    public void doExchangeForNewConnection(Connection con, DTNHost peer) {
        // No metadata exchange required for this router.
    }

    @Override
    public boolean newMessage(Message m) {
        // Accept all newly generated messages.
        return true;
    }

    @Override
    public boolean isFinalDest(Message m, DTNHost aHost) {
        return m.getTo().equals(aHost);
    }

    @Override
    public boolean shouldSaveReceivedMessage(Message m, DTNHost thisHost) {
        // Save the message if this host is not the final destination.
        return !m.getTo().equals(thisHost);
    }

    /**
     * Filters messages for forwarding based on estimated link duration to the peer.
     * By comparing the estimated duration to the threshold.
     * */
    @Override
    public boolean shouldSendMessageToHost(Message m, DTNHost otherHost, DTNHost thisHost) {
        double estimatedDuration = estimateLinkDuration(thisHost, otherHost, detectionRange,
                lastKnownSpeed, lastKnownDirection);
        return estimatedDuration > this.durationThreshold;
    }

    @Override
    public boolean shouldDeleteSentMessage(Message m, DTNHost otherHost) {
        // Do not delete messages after forwarding (epidemic-style replication).
        return false;
    }

    @Override
    public boolean shouldDeleteOldMessage(Message m, DTNHost hostReportingOld) {
        // Do not proactively drop old messages.
        return false;
    }

    /**
     * FIX: update() meng-cache speed DAN direction node saat path aktif.
     * Dipanggil setiap tick — satu-satunya momen reliable untuk membaca
     * getPath() karena path dijamin tidak null di sini.
     */
    @Override
    public void update(DTNHost thisHost) {
        if (thisHost.getPath() != null) {
            // Cache speed
            lastKnownSpeed.put(thisHost, thisHost.getPath().getSpeed());

            // Cache direction: hitung dari current position → next waypoint
            // (sama persis dengan cara yang dipakai di estimateLinkDuration)
            Coord origin = thisHost.getLocation();
            Coord dest   = getNextWaypoint(thisHost);
            double dx    = dest.getX() - origin.getX();
            double dy    = dest.getY() - origin.getY();
            double dist  = Math.sqrt(dx * dx + dy * dy);
            if (dist > 1e-9) {
                lastKnownDirection.put(thisHost, new double[]{ dx / dist, dy / dist });
            }
        }
    }

    @Override
    public RoutingDecisionEngine replicate() {
        return new LinkDurationEstimationRouter(this);
    }

    // -----------------------------------------------------------------------
    // Helper Methods for Angle Calculations
    // -----------------------------------------------------------------------

    /**
     * FIX: Mengambil speed dari Path milik host.
     * Jika path null (pause), gunakan lastKnownSpeed dari cache.
     *
     * @param host  the host to get speed from
     * @param cache map lastKnownSpeed dari instance router ini
     * @return speed valid dalam m/s
     */
    private static double getSpeedFromPath(DTNHost host, Map<DTNHost, Double> cache) {
//        System.out.println("[getSpeedFromPath()] Host " + host + " Path: " + host.getPath());
        if (host.getPath() != null) {
//            System.out.printf("[getSpeedFromPath()] Speed of host %s is: %f%n", host, host.getPath().getSpeed());
            return host.getPath().getSpeed();
        }
//        System.out.printf("[getSpeedFromPath()] The path is null for host %s%n", host);
        // FIX: Gunakan cache speed terakhir saat node sedang pause
        return cache.getOrDefault(host, 0.0);
    }

    /**
     * Menghitung alpha (sudut antara dua velocity vectors).
     * Formula: cos(α) = (V1 · V2) / (√(v1x² + v1y²) × √(v2x² + v2y²))
     *
     * @param v1x x-component dari velocity vector node a
     * @param v1y y-component dari velocity vector node a
     * @param v2x x-component dari velocity vector node b
     * @param v2y y-component dari velocity vector node b
     * @return sudut alpha dalam radian
     */
    private static double calculateAlpha(double v1x, double v1y, double v2x, double v2y) {
        // Hitung magnitude velocity vector secara eksplisit
        double mag1 = Math.sqrt(v1x * v1x + v1y * v1y);
        double mag2 = Math.sqrt(v2x * v2x + v2y * v2y);

        if (mag1 < 1e-9 || mag2 < 1e-9) {
            // One node is paused; alpha is undefined – use pi/2 (worst case)
            return Math.PI / 2.0;
        }

        // Hitung dot product
        double dot = v1x * v2x + v1y * v2y;

        // Hitung cosAlpha menggunakan formula eksplisit
        double cosAlpha = dot / (mag1 * mag2);

        // Clamp to [-1, 1] to guard against floating-point rounding
        cosAlpha = Math.max(-1.0, Math.min(1.0, cosAlpha));
        return Math.acos(cosAlpha);
    }

    /**
     * Menghitung beta (incident angle pada transmission circle).
     *
     * Rumus: β = arcsin(d·sin(θ)/r)
     * Di mana:
     * - d = ||R12|| (magnitude of relative position vector R12 = (x20-x10, y20-y10))
     * - θ = arccos(R12·V12 / (||R12||·||V12||))
     * - r = detectionRadius
     *
     * @param dx x-component dari relative position (R12 = posB - posA)
     * @param dy y-component dari relative position
     * @param v12x x-component dari relative velocity
     * @param v12y y-component dari relative velocity
     * @param relSpeed magnitude dari relative velocity (||V12||)
     * @param detectionRadius transmission range (r)
     * @return sudut beta dalam radian, atau -1.0 jika nodes tidak akan pernah bersentuhan
     */
    private static double calculateBeta(double dx, double dy, double v12x, double v12y, double relSpeed, double detectionRadius) {
        // ================================================================
        // Hitung d = ||R12|| (magnitude of relative position vector)
        // ================================================================
        double d = Math.sqrt(dx * dx + dy * dy);

        // Guard: jika distance 0, return 0
        if (d < 1e-9) {
            return 0.0;
        }

        // ================================================================
        // Hitung θ = arccos(R12·V12 / (||R12||·||V12||))
        // Di mana R12·V12 adalah dot product
        // ================================================================
        double dotProduct = dx * v12x + dy * v12y;

        // Hitung cos(θ)
        double cosTheta = dotProduct / (d * relSpeed);

        // Clamp untuk numerical stability
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));

        // Hitung θ dalam radian
        double theta = Math.acos(cosTheta);

        // ================================================================
        // Hitung β = arcsin(d·sin(θ)/r)
        // ================================================================
        double sinTheta = Math.sin(theta);

        // Hitung argument untuk arcsin: d·sin(θ)/r
        double sinBetaArg = (d * sinTheta) / detectionRadius;

        // Guard: jika argument > 1, berarti nodes tidak akan pernah bersentuhan
        if (sinBetaArg > 1.0) {
            return -1.0; // Sentinel value indicating no contact possible
        }

        // Beta adalah arcsin dari argument
        double beta = Math.asin(sinBetaArg);

        return beta;
    }

    // -----------------------------------------------------------------------
    // Link Duration Estimation (core static method)
    // -----------------------------------------------------------------------

    /**
     * Estimates the remaining link duration between two mobile nodes using the
     * Wu et al. (2009) analytical model.
     *
     * @param a               first node (the "observer" node)
     * @param b               second node
     * @param detectionRadius transmission / detection range in metres
     * @param speedCache      map lastKnownSpeed (fallback saat pause)
     * @param dirCache        map lastKnownDirection (fallback saat pause)
     * @return estimated link duration in seconds, or {@code 0.0} if the link
     *         cannot be established or the nodes are already separating
     */
    public static double estimateLinkDuration(DTNHost a, DTNHost b, double detectionRadius,
                                              Map<DTNHost, Double> speedCache,
                                              Map<DTNHost, double[]> dirCache) {

        // ----------------------------------------------------------------
        // Step 1 – Velocity vectors from current position → next waypoint
        // ----------------------------------------------------------------
        Coord originA = a.getLocation();
        Coord originB = b.getLocation();

        /*
         * Attempt to obtain each node's destination from its movement path.
         * DTNHost.getPath() returns the current movement path; the next
         * waypoint is Path.getNextWaypoint().  If a node has no active path
         * (e.g. it is paused), we fall back to its current position, which
         * produces a zero velocity vector and forces the pause-state branch.
         */
        Coord destA = getNextWaypoint(a);
        Coord destB = getNextWaypoint(b);

        // Velocity (Displacement: Perpindahan) vectors: V = destination - current_position
        double v1x = destA.getX() - originA.getX();
        double v1y = destA.getY() - originA.getY();

        double v2x = destB.getX() - originB.getX();
        double v2y = destB.getY() - originB.getY();

        // ----------------------------------------------------------------
        // Hitung distance displacement untuk mendapatkan direction
        // ----------------------------------------------------------------
        double distA = Math.sqrt(v1x * v1x + v1y * v1y);
        double distB = Math.sqrt(v2x * v2x + v2y * v2y);

        // Normalize ke unit direction vector
        // FIX: Jika distA == 0 (node pause, dest == origin), gunakan lastKnownDirection
        double dirAx, dirAy, dirBx, dirBy;
        if (distA > 1e-9) {
            dirAx = v1x / distA;
            dirAy = v1y / distA;
        } else {
            double[] cached = dirCache.getOrDefault(a, new double[]{ 0.0, 0.0 });
            dirAx = cached[0];
            dirAy = cached[1];
        }
        if (distB > 1e-9) {
            dirBx = v2x / distB;
            dirBy = v2y / distB;
        } else {
            double[] cached = dirCache.getOrDefault(b, new double[]{ 0.0, 0.0 });
            dirBx = cached[0];
            dirBy = cached[1];
        }

        // ================================================================
        // FIX: Ambil speed dari cache jika path sedang null (node pause)
        // ================================================================
        double speedA = getSpeedFromPath(a, speedCache);
        double speedB = getSpeedFromPath(b, speedCache);

        // Velocity = speed * direction unit vector
        double velAx = speedA * dirAx;
        double velAy = speedA * dirAy;
        double velBx = speedB * dirBx;
        double velBy = speedB * dirBy;

        // Update for compatibility with existing code
        v1x = velAx;
        v1y = velAy;
        v2x = velBx;
        v2y = velBy;

        // ----------------------------------------------------------------
        // Step 2 – Relative velocity V12 = V2 - V1
        // ----------------------------------------------------------------
        double v12x = v2x - v1x;
        double v12y = v2y - v1y;
        double relSpeed = Math.sqrt(v12x * v12x + v12y * v12y);

        // ----------------------------------------------------------------
        // Pause-state handling
        // ----------------------------------------------------------------
        if (relSpeed < 1e-9) {
            if (speedA < 1e-9 && speedB < 1e-9) {
                return Double.MAX_VALUE; // both paused – link stays forever
            }

            // ========== ONE NODE STATIONARY, ONE MOVING ==========
            double movingSpeed = (speedA > 1e-9) ? speedA : speedB;

            // Hitung relative position
            double dx = originB.getX() - originA.getX();
            double dy = originB.getY() - originA.getY();
            double d = Math.sqrt(dx * dx + dy * dy);

            // Untuk stationary case, gunakan velocity dari node yang bergerak
            // untuk menghitung beta
            double vMovingx = (speedA > 1e-9) ? v1x : v2x;
            double vMovingy = (speedA > 1e-9) ? v1y : v2y;
            double vMovingMag = Math.sqrt(vMovingx * vMovingx + vMovingy * vMovingy);

            // Guard: jika moving speed masih 0, kembalikan nilai besar
            if (vMovingMag < 1e-9) {
                return detectionRadius * 2.0;
            }

            // Hitung beta untuk stationary case
            double dotProduct = dx * vMovingx + dy * vMovingy;
            double cosTheta = dotProduct / (d * vMovingMag);
            cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
            double theta = Math.acos(cosTheta);
            double sinTheta = Math.sin(theta);
            double sinBetaArg = (d * sinTheta) / detectionRadius;

            // Jika argument > 1, nodes tidak akan pernah bersentuhan
            if (sinBetaArg > 1.0) {
                return 0.0;
            }

            double betaStationary = Math.asin(sinBetaArg);
            double dijStationary = 2.0 * detectionRadius * Math.cos(betaStationary);

            // Link duration = Active distance / moving speed
            double tijStationary = dijStationary / movingSpeed;

            return Math.max(tijStationary, 0.0);
        }

        // ----------------------------------------------------------------
        // Step 3 – Angle alpha between V1 and V2
        // ----------------------------------------------------------------
        double alpha = calculateAlpha(v1x, v1y, v2x, v2y);

        // ----------------------------------------------------------------
        // Step 4 – Relative speed  Vij = sqrt(Vi^2 + Vj^2 - 2*Vi*Vj*cos(alpha))
        // ----------------------------------------------------------------
        double vij = Math.sqrt(speedA * speedA + speedB * speedB - 2.0 * speedA * speedB * Math.cos(alpha));

        if (vij < 1e-9) {
            // Nodes move in lockstep; link is persistent
            return Double.MAX_VALUE;
        }

        // ----------------------------------------------------------------
        // Step 5 – Active distance  Dij = 2r * cos(beta)
        // ----------------------------------------------------------------
        double dx = originB.getX() - originA.getX();
        double dy = originB.getY() - originA.getY();
        double beta = calculateBeta(dx, dy, v12x, v12y, relSpeed, detectionRadius);

        // If perpendicular distance exceeds r the nodes will never enter each
        // other's transmission range along the current trajectories.
        if (beta < 0.0) {
            return 0.0;
        }

        double dij = 2.0 * detectionRadius * Math.cos(beta);

        // ----------------------------------------------------------------
        // Step 6 – Link duration  Tij = Dij / Vij
        // ----------------------------------------------------------------
        double tij = dij / vij;

        return Math.max(tij, 0.0);
    }

    // -----------------------------------------------------------------------
    // Helper utilities
    // -----------------------------------------------------------------------

    /**
     * Returns the next waypoint destination of a host WITHOUT advancing the
     * path's internal index, or the current location if it has no active path.
     */
    private static Coord getNextWaypoint(DTNHost host) {
        if (host.getPath() != null) {
            java.util.List<Coord> coords = host.getPath().getCoords();
            if (!coords.isEmpty()) {
                return coords.get(coords.size() - 1);
            }
        }
        return host.getLocation();
    }

    public double getDetectionRange() {
        return this.detectionRange;
    }

    /** Getter untuk speedCache — digunakan oleh LinkDurationComparisonReport. */
    public Map<DTNHost, Double> getLastKnownSpeed() {
        return lastKnownSpeed;
    }

    /** Getter untuk directionCache — digunakan oleh LinkDurationComparisonReport. */
    public Map<DTNHost, double[]> getLastKnownDirection() {
        return lastKnownDirection;
    }
}
