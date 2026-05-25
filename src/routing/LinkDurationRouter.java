package routing;

import core.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LinkDurationRouter extends ActiveRouter {

    public static final String LDR_NS = "LinkDurationRouter";
    public static final String DURATION_THRESHOLD_S = "durationThreshold";

    private final double durationThreshold ;

    // Last Positions
    private Map<DTNHost, double[]> lastPositions = new HashMap<>();
    private Map<DTNHost, Double> lastTimes = new HashMap<>();
    private Map<DTNHost, Double> estimatedDurations = new HashMap<DTNHost, Double>();

    public LinkDurationRouter(Settings s) {
        super(s);
        Settings LDR = new Settings(LDR_NS);

        this.durationThreshold = LDR.getDouble(DURATION_THRESHOLD_S, 5);
    }

    public LinkDurationRouter(LinkDurationRouter r) {
        super(r);
        this.durationThreshold = r.durationThreshold;
    }

    /**
     * Menghitung kecepatan node berdasarkan perubahan posisi.
     * Velocity vector = [vx, vy] dari perubahan posisi per unit waktu
     */
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

    /**
     * Estimasi Link Duration berdasarkan paper:
     * Wu, Y.-T., Liao, W., Tsao, C.-L., & Lin, T.-N. (2009).
     * "Impact of Node Mobility on Link Duration in Multihop Mobile Networks"
     *
     * Formula: Tij = Dij / Vij
     * - Dij : Active Distance = 2r·cos(βij)
     * - Vij : Relative Speed = √(Vi² + Vj² - 2·Vi·Vj·cos(αij))
     */
    protected double estimateLinkDuration(DTNHost a, DTNHost b) {

        // ========== LANGKAH 1: Hitung Kecepatan (Vi dan Vj) ==========
        double[] va = getVelocity(a);
        double[] vb = getVelocity(b);

        double vi = Math.sqrt(va[0]*va[0] + va[1]*va[1]);
        double vj = Math.sqrt(vb[0]*vb[0] + vb[1]*vb[1]);

        // Guard: jika velocity belum valid (awal simulasi), return 0
        if (vi == 0 && vj == 0) {
            return 0;
        }

        // ========== LANGKAH 2: Hitung Sudut α (cosine angle antara kedua velocity vectors) ==========
        double dot = (va[0]*vb[0] + va[1]*vb[1]);
        double cosAlpha = (vi != 0 && vj != 0) ? dot / (vi * vj) : 0;

        // ========== LANGKAH 3: Hitung Relative Speed (Vij) ==========
        // Formula: Vij = √(Vi² + Vj² - 2·Vi·Vj·cos(αij))
        double Vij = Math.sqrt(vi*vi + vj*vj - 2 * vi * vj * cosAlpha);

        // Guard: jika relative speed hampir 0, kedua node bergerak kearah yang sama
        if (Vij < 1e-6) {
            return Double.MAX_VALUE;
        }

        // ========== LANGKAH 4: Hitung Jarak Antar Node (d) ==========
        double dx = a.getLocation().getX() - b.getLocation().getX();
        double dy = a.getLocation().getY() - b.getLocation().getY();
        double d = Math.sqrt(dx*dx + dy*dy);

        double r = getHost().getInterfaces().get(0).getTransmitRange();

        // Guard: jika jarak melebihi 2r, note tidak terhubung
        if (d >= 2 * r) return 0;

        // ========== LANGKAH 5: Hitung Sudut β (incident angle) ==========
        // Dari geometri: cos(β) = √(r² - (d/2)²) / r
        double cosBeta = Math.sqrt(Math.max(0, r*r - (d*d / 4))) / r;

        // ========== LANGKAH 6: Hitung Active Distance (Dij) ==========
        // Formula: Dij = 2r·cos(βij)
        double Dij = 2 * r * cosBeta;

        // ========== LANGKAH 7: Hitung Link Duration (Tij) ==========
        // Formula: Tij = Dij / Vij
        return Dij / Vij;
    }

    /**
     * Filter koneksi yang tersedia berdasarkan estimasi link duration.
     * Hanya koneksi dengan estimasi durasi >= durationThreshold yang disertakan.
     *
     * Tujuan: Meminimalkan terjadinya koneksi yang terputus di tengah transfer.
     */
    protected List<Connection> filterConnections(List<Connection> connections) {

        List<Connection> filtered = new ArrayList<>();

        for (Connection con : connections) {
            DTNHost other = con.getOtherNode(getHost());

            double duration = estimateLinkDuration(getHost(), other);
            estimatedDurations.put(other, duration);

            if (duration >= durationThreshold) {
                filtered.add(con);
            }
        }

        return filtered;
    }
    public Map<DTNHost, Double> getEstimatedDurations() {return this.estimatedDurations;}


    @Override
    public void update() {
        super.update();

        // PENCEGAHAN KONFLIK: Jangan jalankan Epidemic transfer jika diwarisi oleh Knapsack Router
        if (this.getClass() != LinkDurationRouter.class && this.getClass() != EpidemicRouterMod.class) {
            return;
        }

        if (isTransferring() || !canStartTransfer()) {
            return; // transferring, don't try other connections yet
        }

        // Try first the messages that can be delivered to final recipient
        if (exchangeDeliverableMessages() != null) {
            return; // started a transfer, don't try others (yet)
        }

        // then try any/all message to any/all connection
        this.tryMessagesToConnections(new ArrayList<>(getMessageCollection()),filterConnections(getConnections()));
    }

    @Override
    public LinkDurationRouter replicate() {
        return new LinkDurationRouter(this);
    }
}