package routing;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Message;
import core.Settings;
import core.SimClock;
import routing.rapid.DelayEntry;
import routing.rapid.DelayTable;
import routing.rapid.MeetingEntry;
import routing.community.Duration;

public class KnapsackDPRouter implements RoutingDecisionEngine {

    public static final String LDER_NS = "KDPR";

    /** Settings key for detection/transmission range (metres). */
    public static final String DETECTION_RANGE_SETTING = "detectionRange";

    /** Default detection range in metres if not specified in settings. */
    private static final double DEFAULT_DETECTION_RANGE = 10.0;

    /** Transmission / detection range of this node. */
    private double detectionRange;

    private double durationThreshold;

//    private int[] minAndMaxSpeed;

    // -----------------------------------------------------------------------
    // Knapsack DP State Variables
    // -----------------------------------------------------------------------

    private DelayTable delayTable;
    private final Map<DTNHost, Integer> retrictionPerPeer = new HashMap<>();
    private final LinkedList<Integer> lengthMsg = new LinkedList<>();
    private final LinkedList<Double> utilityMsg = new LinkedList<>();
    private final LinkedList<Message> tempMsg = new LinkedList<>();
    private final LinkedList<Message> tempMsgTerpilih = new LinkedList<>();
    protected final Map<DTNHost, Double> startTimestamps = new HashMap<>();
    protected final Map<DTNHost, List<Duration>> connHistory = new HashMap<>();

    private final UtilityAlgorithm ALGORITHM = UtilityAlgorithm.AVERAGE_DELAY;
    private static final double INFINITY = 99999.0;
    private static final int BYTES_UNIT = 1000; // bytes per unit for knapsack capacity
    private static final double UTILITY_INTERVAL = 100.0; // interval to verify connections in seconds

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

    public KnapsackDPRouter(Settings _settings) {
        Settings s = new Settings(LDER_NS);

//        this.minAndMaxSpeed = s.getCsvInts("speed", 2);

//        this.detectionRange = s.getDouble(DETECTION_RANGE_SETTING, DEFAULT_DETECTION_RANGE);
        this.detectionRange = (new Settings("btInterface")).getDouble("transmitRange", DEFAULT_DETECTION_RANGE);
        this.durationThreshold = s.getDouble("durationThreshold", 0);
    }

    public KnapsackDPRouter(KnapsackDPRouter proto) {
//        this.minAndMaxSpeed = proto.minAndMaxSpeed;
        this.detectionRange = proto.detectionRange;
        this.durationThreshold = proto.durationThreshold;
        this.delayTable = proto.delayTable;
        // Do not copy caches — each node instance has its own router.
        // Maps will be refilled by update() and connectionUp() from start of simulation.
    }

    // -----------------------------------------------------------------------
    // RoutingDecisionEngine interface
    // -----------------------------------------------------------------------

    @Override
    public void connectionUp(DTNHost thisHost, DTNHost peer) {
        // Initialize connection timing
        this.startTimestamps.put(peer, SimClock.getTime());
        
        // Exchange delay table metadata with peer
        KnapsackDPRouter peerRouter = getKnapsackDPRouter(peer);
        if (peerRouter != null) {
            // Sync delay tables bidirectionally
            synchronizeDelayTables(this, peerRouter);
            synchronizeMeetingTimes(this, peerRouter);
            synchronizeAckedMessageIDs(this, peerRouter);
        }
        
        // Compute knapsack capacity based on link duration
        double estimatedDuration = LinkDurationEstimationRouter.estimateLinkDuration(
            thisHost, peer, this.detectionRange, this.lastKnownSpeed, this.lastKnownDirection);
        // Capacity = duration / transfer_time_per_byte; approximate as duration / 0.1 seconds
        int capacity = (int) (estimatedDuration / 0.1);
        this.retrictionPerPeer.put(peer, Math.max(1, capacity));
    }

    @Override
    public void connectionDown(DTNHost thisHost, DTNHost peer) {
        // Record connection duration
        if (this.startTimestamps.containsKey(peer)) {
            double startTime = this.startTimestamps.get(peer);
            double endTime = SimClock.getTime();
            Duration d = new Duration(startTime, endTime);
            
            if (!this.connHistory.containsKey(peer)) {
                this.connHistory.put(peer, new ArrayList<>());
            }
            this.connHistory.get(peer).add(d);
            
            this.startTimestamps.remove(peer);
        }
        
        // Remove peer from knapsack capacity map
        this.retrictionPerPeer.remove(peer);
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
     * Filters messages for forwarding based on:
     * 1. Estimated link duration must exceed threshold
     * 2. Message must be selected by knapsack 0/1 DP algorithm
     * */
    @Override
    public boolean shouldSendMessageToHost(Message m, DTNHost otherHost, DTNHost thisHost) {
        // Step 1: Check link duration threshold
        double estimatedDuration = LinkDurationEstimationRouter.estimateLinkDuration(
            thisHost, otherHost, this.detectionRange,
            lastKnownSpeed, lastKnownDirection);
        if (estimatedDuration <= this.durationThreshold) {
            return false; // Link not viable
        }
        
        // Step 2: Check knapsack capacity
        if (!this.retrictionPerPeer.containsKey(otherHost)) {
            return true; // No capacity restriction, send freely
        }
        
        int knapsackCapacity = this.retrictionPerPeer.get(otherHost);
        if (knapsackCapacity <= 0) {
            return false; // No capacity
        }
        
        // Step 3: Collect all messages, compute utilities, and solve knapsack
        Collection<Message> allMessages = thisHost.getMessageCollection();
        List<Integer> sizes = new ArrayList<>();
        List<Double> utilities = new ArrayList<>();
        List<Message> messageList = new ArrayList<>(allMessages);
        
        for (Message msg : messageList) {
            sizes.add(msg.getSize() / BYTES_UNIT);
            utilities.add(computeUtility(msg, otherHost, true));
        }
        
        // Step 4: Solve 0/1 knapsack
        boolean[] selected = knapsackSolve(sizes, utilities, knapsackCapacity);
        
        // Step 5: Check if current message is in the solution
        int msgIndex = messageList.indexOf(m);
        return msgIndex >= 0 && msgIndex < selected.length && selected[msgIndex];
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
     * NEW: Also maintain delay table and delete acked messages.
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
        
        // Initialize DelayTable if needed
        if (this.delayTable == null && thisHost != null) {
            this.delayTable = new DelayTable(thisHost);
        }
    }

    @Override
    public RoutingDecisionEngine replicate() {
        return new KnapsackDPRouter(this);
    }

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

    private KnapsackDPRouter getKnapsackDPRouter(DTNHost host) {
        if (host == null) {
            return null;
        }
        MessageRouter router = host.getRouter();
        if (router instanceof DecisionEngineRouter) {
            RoutingDecisionEngine engine = ((DecisionEngineRouter) router).getDecisionEngine();
            if (engine instanceof KnapsackDPRouter) {
                return (KnapsackDPRouter) engine;
            }
        }
        return null;
    }

    // -----------------------------------------------------------------------
    // Knapsack DP Helper Methods
    // -----------------------------------------------------------------------

    /**
     * Compute utility for a message if sent to a peer.
     * Utility = -EstimatedDelay (lower delay = higher utility)
     */
    private double computeUtility(Message msg, DTNHost host, boolean recompute) {
        double utility = 0.0;
        double packetDelay = 0.0;

        switch (ALGORITHM) {
            case AVERAGE_DELAY: {
                packetDelay = estimateDelay(msg, host, recompute);
                utility = -packetDelay;
                break;
            }
            case MISSED_DEADLINES: {
                double lifeTime = msg.getTtl() * 60; // in seconds
                double timeSinceCreation = SimClock.getTime() - msg.getCreationTime();
                double remainingTime = computeRemainingTime(msg);
                if (lifeTime > timeSinceCreation) {
                    if (remainingTime < lifeTime - timeSinceCreation) {
                        utility = lifeTime - timeSinceCreation - remainingTime;
                    } else {
                        utility = 0;
                    }
                } else {
                    utility = 0;
                }
                break;
            }
            case MAXIMUM_DELAY: {
                packetDelay = estimateDelay(msg, host, recompute);
                // Check if this message has the maximum delay
                Collection<Message> allMessages = host.getMessageCollection();
                boolean isMaxDelay = true;
                for (Message other : allMessages) {
                    if (!other.equals(msg)) {
                        if (packetDelay < estimateDelay(other, host, recompute)) {
                            isMaxDelay = false;
                            break;
                        }
                    }
                }
                utility = isMaxDelay ? -packetDelay : 0;
                break;
            }
            default:
                utility = 0;
        }
        return utility;
    }

    /**
     * Estimate delay for message delivery to a host.
     */
    private double estimateDelay(Message msg, DTNHost host, boolean recompute) {
        double remainingTime = 0.0;
        double packetDelay = 0.0;

        if (this.delayTable == null) {
            return INFINITY;
        }

        if ((recompute) && ((this.delayTable.delayHasChanged(msg.getId())) 
                || (this.delayTable.getDelayEntryByMessageId(msg.getId()) == null)
                || (this.delayTable.getDelayEntryByMessageId(msg.getId()).getDelayOf(host) == null))) {
            remainingTime = computeRemainingTime(msg);
            packetDelay = Math.min(this.INFINITY, computePacketDelay(msg, host, remainingTime));
        } else if (this.delayTable.getDelayEntryByMessageId(msg.getId()) != null) {
            Double delay = this.delayTable.getDelayEntryByMessageId(msg.getId()).getDelayOf(host);
            packetDelay = delay != null ? delay : INFINITY;
        } else {
            packetDelay = INFINITY;
        }

        return packetDelay;
    }

    /**
     * Compute remaining time for message delivery.
     */
    private double computeRemainingTime(Message msg) {
        double transferTime = this.INFINITY;
        double remainingTime = computeTransferTime(msg, msg.getTo());

        if (this.delayTable != null && this.delayTable.getDelayEntryByMessageId(msg.getId()) != null) {
            for (Entry<DTNHost, ?> entry : this.delayTable.getDelayEntryByMessageId(msg.getId()).getDelays()) {
                DTNHost host = entry.getKey();
                KnapsackDPRouter peerRouter = getKnapsackDPRouter(host);
                if (peerRouter != null) {
                    transferTime = peerRouter.computeTransferTime(msg, msg.getTo());
                    remainingTime = Math.min(transferTime, remainingTime);
                }
            }
        }

        return remainingTime;
    }

    /**
     * Compute transfer time for a message to reach destination.
     */
    private double computeTransferTime(Message msg, DTNHost host) {
        if (this.delayTable == null) {
            return INFINITY;
        }

        double packetsSize = 0.0;
        double transferOpportunity = this.delayTable.getAvgTransferOpportunity();
        double meetingTime = 0.0;
        double transferTime = this.INFINITY;

        // Sum sizes of other messages
        Collection<Message> allMessages = host.getMessageCollection();
        for (Message m : allMessages) {
            if (!m.equals(msg)) {
                packetsSize += m.getSize();
            }
        }

        // Get meeting entry
        MeetingEntry entry = this.delayTable.getIndirectMeetingEntry(host.getAddress(), host.getAddress());
        if (entry != null) {
            meetingTime = entry.getAvgMeetingTime();
            transferTime = meetingTime * Math.ceil(packetsSize / Math.max(1.0, transferOpportunity));
        }

        return transferTime;
    }

    /**
     * Compute packet delay combining creation time and remaining delivery time.
     */
    private double computePacketDelay(Message msg, DTNHost host, double remainingTime) {
        double timeSinceCreation = SimClock.getTime() - msg.getCreationTime();
        double packetDelay = timeSinceCreation + remainingTime;
        return packetDelay;
    }

    /**
     * Get knapsack capacity restriction for sending to a peer.
     */
    private int getRestrictionForSend(DTNHost thisHost, DTNHost otherHost) {
        return this.retrictionPerPeer.getOrDefault(otherHost, 0);
    }

    /**
     * Solve 0/1 knapsack problem using dynamic programming.
     * Returns a boolean array where selected[i] = true if item i is selected.
     */
    public boolean[] knapsackSolve(List<Integer> sizes, List<Double> utilities, int capacity) {
        int n = sizes.size();
        if (n == 0 || capacity <= 0) {
            return new boolean[n];
        }

        // Transform negative utilities to positive
        double maxNegative = 0;
        for (Double u : utilities) {
            if (u < maxNegative) {
                maxNegative = u;
            }
        }

        // DP table
        double[][] dp = new double[n + 1][capacity + 1];
        for (int i = 0; i <= n; i++) {
            for (int w = 0; w <= capacity; w++) {
                if (i == 0 || w == 0) {
                    dp[i][w] = 0;
                } else if (sizes.get(i - 1) <= w) {
                    double transformedProfit = utilities.get(i - 1) - maxNegative;
                    dp[i][w] = Math.max(
                        dp[i - 1][w],
                        transformedProfit + dp[i - 1][w - sizes.get(i - 1)]
                    );
                } else {
                    dp[i][w] = dp[i - 1][w];
                }
            }
        }

        // Backtrack to find selected items
        boolean[] selected = new boolean[n];
        int w = capacity;
        for (int i = n; i >= 1 && w > 0; i--) {
            if (dp[i][w] > dp[i - 1][w]) {
                selected[i - 1] = true;
                w -= sizes.get(i - 1);
            }
        }

        return selected;
    }

    /**
     * Synchronize delay tables bidirectionally with peer.
     */
    private void synchronizeDelayTables(KnapsackDPRouter thisRouter, KnapsackDPRouter peerRouter) {
        if (this.delayTable == null || peerRouter.delayTable == null) {
            return;
        }

        // Sync this router's delay entries to peer
        List<DelayEntry> myEntries = new ArrayList<>();
        for (Entry<String, DelayEntry> entry : this.delayTable.getDelayEntries()) {
            myEntries.add(entry.getValue());
        }
        
        for (DelayEntry myEntry : myEntries) {
            if (peerRouter.delayTable.getDelayEntryByMessageId(myEntry.getMessage().getId()) == null) {
                peerRouter.delayTable.addEntry(myEntry);
            }
        }
        
        // Reverse sync: peer's delay entries to this router
        List<DelayEntry> peerEntries = new ArrayList<>();
        for (Entry<String, DelayEntry> entry : peerRouter.delayTable.getDelayEntries()) {
            peerEntries.add(entry.getValue());
        }
        
        for (DelayEntry peerEntry : peerEntries) {
            if (this.delayTable.getDelayEntryByMessageId(peerEntry.getMessage().getId()) == null) {
                this.delayTable.addEntry(peerEntry);
            }
        }
    }

    /**
     * Synchronize meeting times bidirectionally with peer.
     */
    private void synchronizeMeetingTimes(KnapsackDPRouter thisRouter, KnapsackDPRouter peerRouter) {
        if (this.delayTable == null || peerRouter.delayTable == null) {
            return;
        }

        // Sync meeting entries - simplified version
        int dimension = this.delayTable.getMeetingMatrixDimension();
        for (int i = 0; i < dimension; i++) {
            for (int j = 0; j < dimension; j++) {
                MeetingEntry myEntry = this.delayTable.getMeetingEntry(i, j);
                if (myEntry != null) {
                    // Meeting entries are updated during connections
                }
            }
        }
    }

    /**
     * Synchronize acked message IDs bidirectionally with peer.
     */
    private void synchronizeAckedMessageIDs(KnapsackDPRouter thisRouter, KnapsackDPRouter peerRouter) {
        if (this.delayTable == null || peerRouter.delayTable == null) {
            return;
        }

        // Exchange acked message IDs
        this.delayTable.addAllAckedMessageIds(peerRouter.delayTable.getAllAckedMessageIds());
        peerRouter.delayTable.addAllAckedMessageIds(this.delayTable.getAllAckedMessageIds());
    }

    /**
     * Utility algorithm selection enum.
     */
    private enum UtilityAlgorithm {
        AVERAGE_DELAY,
        MISSED_DEADLINES,
        MAXIMUM_DELAY;
    }
}
