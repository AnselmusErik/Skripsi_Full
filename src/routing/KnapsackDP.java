/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package routing;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import routing.rapid.DelayEntry;
import routing.rapid.DelayTable;
import routing.rapid.MeetingEntry;

import core.*;
import java.text.DecimalFormat;
import java.util.Iterator;
import java.util.LinkedList;
import movement.Path;
import static routing.MessageRouter.RCV_OK;
import routing.community.Duration;

/**
 *
 * @author Kalis, Universitas Sanata Dharma
 */
public class KnapsackDP extends LinkDurationRouter {
    // timestamp for meeting a host in seconds

    private double timestamp;
    // utility algorithm (minimizing average delay | minimizing missed deadlines |
    // minimizing maximum delay)
    private Map<Integer, DTNHost> hostMapping;

    private UtilityAlgorithm algorithm = UtilityAlgorithm.AVERAGE_DELAY;
    private static final double INFINITY = 99999;

    // Prophet variables
    public static final double P_INIT = 0.75;
    public static final double DEFAULT_BETA = 0.25;
    public static final double GAMMA = 0.98;
    public static final String PROPHET_NS = "ProphetRouter";
    public static final String SECONDS_IN_UNIT_S = "secondsInTimeUnit";
    public static final String BETA_S = "beta";

    private int secondsInTimeUnit;
    private double beta;
    private Map<DTNHost, Double> preds;
    private double lastAgeUpdate;

    // interval to verify ongoing connections in seconds
    private static final double UTILITY_INTERAL = 100.0;
    private DelayTable delayTable;

    protected Map<DTNHost, Double> startTimestamps;
    protected Map<DTNHost, List<Duration>> connHistory;
    LinkedList<Integer> lengthMsg;
    LinkedList<Double> utilityMsg;
    LinkedList<Message> tempMsg;
    LinkedList<Message> tempMsgTerpilih;
    Map<DTNHost, Integer> retrictionPerPeer;
    private DTNHost getOtherHost;
    private static final int bytes = 1000;

    private final Map<DTNHost, Double> lastKnownSpeed = new HashMap<>();
    private final Map<DTNHost, double[]> lastKnownDirection = new HashMap<>();

    private static Coord getNextWaypoint(DTNHost host) {
        if (host.getPath() != null) {
            java.util.List<Coord> coords = host.getPath().getCoords();
            if (!coords.isEmpty()) {
                return coords.get(coords.size() - 1);
            }
        }
        return host.getLocation();
    }

    /**
     * Constructor. Creates a new message router based on the settings in the
     * given Settings object.
     *
     * @param s The settings object
     */
    public KnapsackDP(Settings s) {
        super(s);

        delayTable = null;
        timestamp = 0.0;
        hostMapping = new HashMap<Integer, DTNHost>();
        startTimestamps = new HashMap<DTNHost, Double>();
        connHistory = new HashMap<DTNHost, List<Duration>>();
        lengthMsg = new LinkedList<Integer>();
        utilityMsg = new LinkedList<Double>();
        tempMsg = new LinkedList<Message>();
        tempMsgTerpilih = new LinkedList<Message>();
        retrictionPerPeer = new HashMap<DTNHost, Integer>();
        getOtherHost = null;

        if (s.contains("utilityMode")) {
            String modeStr = s.getSetting("utilityMode");
            try {
                this.algorithm = UtilityAlgorithm.valueOf(modeStr.toUpperCase());
            } catch (IllegalArgumentException e) {
                this.algorithm = UtilityAlgorithm.AVERAGE_DELAY;
            }
        } else {
            this.algorithm = UtilityAlgorithm.AVERAGE_DELAY;
        }

        initProphet(s);
    }

    @Override
    public void init(DTNHost host, List<MessageListener> mListeners) {
        super.init(host, mListeners);
        delayTable = new DelayTable(host);
        // System.out.println(host);
    }

    /**
     * Copy constructor.
     *
     * @param r The router prototype where setting values are copied from
     */
    protected KnapsackDP(KnapsackDP r) {
        super(r);

        delayTable = r.delayTable;
        timestamp = r.timestamp;
        hostMapping = r.hostMapping;
        startTimestamps = r.startTimestamps;
        connHistory = r.connHistory;
        lengthMsg = r.lengthMsg;
        utilityMsg = r.utilityMsg;
        tempMsg = r.tempMsg;
        tempMsgTerpilih = r.tempMsgTerpilih;
        getOtherHost = r.getOtherHost;
        retrictionPerPeer = r.retrictionPerPeer;
        this.algorithm = r.algorithm;

        this.secondsInTimeUnit = r.secondsInTimeUnit;
        this.beta = r.beta;
        initPreds();
    }

    @Override
    public void changedConnection(Connection con) {
        if (con.isUp()) {

            DTNHost otherHost = con.getOtherNode(getHost());
            this.getOtherHost = null;
            this.getOtherHost = otherHost;
            this.startTimestamps.put(otherHost, SimClock.getTime());

            /* new connection */
            // simulate control channel on connection up without sending any data
            this.timestamp = SimClock.getTime();

            // synchronize all delay table entries
            synchronizeDelayTables(con);

            // synchronize all meeting time entries
            synchronizeMeetingTimes(con);
            updateDelayTableStat(con);

            // synchronize acked message ids
            synchronizeAckedMessageIDs(con);
            deleteAckedMessages();
            // map DTNHost to their address
            doHostMapping(con);
            this.delayTable.dummyUpdateConnection(con);

            if (this.algorithm == UtilityAlgorithm.PROPHET) {
                updateDeliveryPredFor(otherHost);
                updateTransitivePreds(otherHost);
            }

            // Perhitungan Estimate Time Duration untuk Kapasitas Knapsack
            if (!String.valueOf(getHost().toString().charAt(0)).equals("s")) { // Pengecekan apakah dia Sensor atau
                                                                               // bukan
                // Menghitung batas ETD (Estimated Time Duration) secara dinamis dengan LDER baru
                double duration = LinkDurationEstimationRouter.estimateLinkDuration(
                        getHost(), otherHost, getHost().getInterfaces().get(0).getTransmitRange(),
                        this.lastKnownSpeed, this.lastKnownDirection);
                System.out.println("Duration: " + duration);
                double speed = getHost().getInterfaces().get(0).getTransmitSpeed();
                
                int capacityOFKnapsack = (int) ((duration * speed) / bytes);
                int maxCapacity = (int) (otherHost.getRouter().getBufferSize() / bytes);
                if (capacityOFKnapsack > maxCapacity || capacityOFKnapsack < 0 || duration == Double.MAX_VALUE) {
                    capacityOFKnapsack = maxCapacity;
                }


                this.retrictionPerPeer.put(otherHost, capacityOFKnapsack); // Nilai kapasitas disimpan dalam Map
                                                                           // (Batasan)
                cekSyaratKnapsackSend(getHost(), otherHost); // Cek syarat untuk pengiriman pesan
            }
        } else {
            /* connection went down */
            // update connection
            double time = SimClock.getTime() - this.timestamp;
            this.delayTable.updateConnection(con, time);

            synchronizeAckedMessageIDs(con);
            deleteAckedMessages();
            updateAckedMessageIds();
            synchronizeDelayTables(con);

            DTNHost otherHost = con.getOtherNode(getHost());
            double times = 0;
            if (this.startTimestamps.containsKey(otherHost)) {
                times = this.startTimestamps.get(otherHost);
            }

            double etime = SimClock.getTime();
            List<Duration> history;
            if (!this.connHistory.containsKey(otherHost)) {
                history = new LinkedList<Duration>();
                this.connHistory.put(otherHost, history);
            } else {
                history = this.connHistory.get(otherHost);
            }

            // add this connection to the list
            if (etime - times > 0) {
                history.add(new Duration(times, etime));
            }

            this.connHistory.put(otherHost, history);
            this.startTimestamps.remove(otherHost);

            this.lengthMsg.clear();
            this.utilityMsg.clear();
            this.tempMsg.clear();
            this.tempMsgTerpilih.clear();
        }
    }

    private void doHostMapping(Connection con) {
        DTNHost host = getHost();
        DTNHost otherHost = con.getOtherNode(host);
        KnapsackDP otherRouter = ((KnapsackDP) otherHost.getRouter());

        // propagate host <-> address mapping
        this.hostMapping.put(host.getAddress(), host);
        this.hostMapping.put(otherHost.getAddress(), otherHost);
        this.hostMapping.putAll(otherRouter.hostMapping);
        otherRouter.hostMapping.putAll(this.hostMapping);
    }

    private void updateDelayTableStat(Connection con) {
        DTNHost otherHost = con.getOtherNode(getHost());
        if (!(otherHost.getRouter() instanceof KnapsackDP)) {
            return;
        }
        KnapsackDP otherRouter = (KnapsackDP) otherHost.getRouter();
        if (otherRouter.getDelayTable() == null) {
            return;
        }
        int from = otherHost.getAddress();

        for (Message m : getMessageCollection()) {
            int to = m.getTo().getAddress();
            MeetingEntry entry = otherRouter.getDelayTable().getMeetingEntry(from, to);
            if (entry != null) {
                routing.rapid.DelayEntry de = this.delayTable.getDelayEntryByMessageId(m.getId());
                if (de != null) {
                    de.setChanged(true);
                }
            }
        }
    }

    private void synchronizeDelayTables(Connection con) {
        DTNHost otherHost = con.getOtherNode(getHost());
        KnapsackDP otherRouter = (KnapsackDP) otherHost.getRouter();
        DelayEntry delayEntry = null;
        DelayEntry otherDelayEntry = null;

        // synchronize all delay table entries
        for (Map.Entry<String, DelayEntry> entry1 : this.delayTable.getDelayEntries()) {
            Message m = entry1.getValue().getMessage();
            delayEntry = this.delayTable.getDelayEntryByMessageId(m.getId());
            assert (delayEntry != null);
            otherDelayEntry = otherRouter.delayTable.getDelayEntryByMessageId(m.getId());

            if (delayEntry.getDelays() == null) {
                continue;
            }
            // for all hosts check delay entries and create new if they doesn't exist
            for (Map.Entry<DTNHost, Tuple<Double, Double>> entry : delayEntry.getDelays()) {
                DTNHost myHost = entry.getKey();
                Double myDelay = entry.getValue().getKey();
                Double myTime = entry.getValue().getValue();

                // create a new host entry if host entry at other host doesn't exist
                if ((otherDelayEntry == null) || (!otherDelayEntry.contains(myHost))) {
                    // parameters:
                    // m The message
                    // myHost The host which contains a copy of this message
                    // myDelay The estimated delay
                    // myTime The entry was last changed
                    otherRouter.updateDelayTableEntry(m, myHost, myDelay, myTime);
                } else {
                    // check last update time of other hosts entry and update it
                    if (otherDelayEntry.isOlderThan(myHost, delayEntry.getLastUpdate(myHost))) {
                        // parameters:
                        // m The message
                        // myHost The host which contains a copy of this message
                        // myDelay The estimated delay
                        // myTime The entry was last changed

                        otherRouter.updateDelayTableEntry(m, myHost, myDelay, myTime);
                    }

                    if ((otherDelayEntry.isAsOldAs(myHost, delayEntry.getLastUpdate(myHost)))
                            && (delayEntry.getDelayOf(myHost) > otherDelayEntry.getDelayOf(myHost))) {
                        // parameters:
                        // m The message
                        // myHost The host which contains a copy of this message
                        // myDelay The estimated delay
                        // myTime The entry was last changed

                        otherRouter.updateDelayTableEntry(m, myHost, myDelay, myTime);
                    }
                }
            }
        }
    }

    private void synchronizeMeetingTimes(Connection con) {
        DTNHost otherHost = con.getOtherNode(getHost());
        KnapsackDP otherRouter = (KnapsackDP) otherHost.getRouter();
        MeetingEntry meetingEntry = null;
        MeetingEntry otherMeetingEntry = null;

        // synchronize all meeting time entries
        for (int i = 0; i < this.delayTable.getMeetingMatrixDimension(); i++) {
            for (int k = 0; k < this.delayTable.getMeetingMatrixDimension(); k++) {
                meetingEntry = this.delayTable.getMeetingEntry(i, k);

                if (meetingEntry != null) {
                    otherMeetingEntry = otherRouter.delayTable.getMeetingEntry(i, k);
                    // create a new meeting entry if meeting entry at other host doesn't exist
                    if (otherMeetingEntry == null) {
                        otherRouter.delayTable.setAvgMeetingTime(i, k, meetingEntry.getAvgMeetingTime(),
                                meetingEntry.getLastUpdate(), meetingEntry.getWeight());
                    } else {
                        // check last update time of other hosts entry and update it
                        if (otherMeetingEntry.isOlderThan(meetingEntry.getLastUpdate())) {
                            otherRouter.delayTable.setAvgMeetingTime(i, k, meetingEntry.getAvgMeetingTime(),
                                    meetingEntry.getLastUpdate(), meetingEntry.getWeight());
                        }

                        if ((otherMeetingEntry.isAsOldAs(meetingEntry.getLastUpdate()))
                                && (meetingEntry.getAvgMeetingTime() > otherMeetingEntry.getAvgMeetingTime())) {
                            otherRouter.delayTable.setAvgMeetingTime(i, k, meetingEntry.getAvgMeetingTime(),
                                    meetingEntry.getLastUpdate(), meetingEntry.getWeight());
                        }
                    }
                }
            }
        }
    }

    private void synchronizeAckedMessageIDs(Connection con) {
        DTNHost otherHost = con.getOtherNode(getHost());
        KnapsackDP otherRouter = (KnapsackDP) otherHost.getRouter();

        this.delayTable.addAllAckedMessageIds(otherRouter.delayTable.getAllAckedMessageIds());
        otherRouter.delayTable.addAllAckedMessageIds(this.delayTable.getAllAckedMessageIds());

        assert (this.delayTable.getAllAckedMessageIds().equals(otherRouter.delayTable.getAllAckedMessageIds()));
    }

    /**
     * Deletes the messages from the message buffer that are known to be ACKed
     * and also delete the message entries of the ACKed messages in the delay
     * table
     */
    private void deleteAckedMessages() {
        for (String id : this.delayTable.getAllAckedMessageIds()) {

            // delete messages from the delay table
            if (this.delayTable.getDelayEntryByMessageId(id) != null) {
                assert (this.delayTable.removeEntry(id) == true);
            }
        }
    }

    @Override
    public Message messageTransferred(String id, DTNHost from) {
        Message m = super.messageTransferred(id, from);

        /* was this node the final recipient of the message? */
        if (isDeliveredMessage(m)) {
            this.delayTable.addAckedMessageIds(id);
        }
        return m;
    }

    /**
     * Delete old message ids from the acked message id set which time to live
     * was passed
     */
    private void updateAckedMessageIds() {
        ArrayList<String> removableIds = new ArrayList<String>();

        for (String id : this.delayTable.getAllAckedMessageIds()) {
            Message m = this.getMessage(id);
            if ((m != null) && (m.getTtl() <= 0)) {
                removableIds.add(id);
            }
        }

        if (removableIds.size() > 0) {
            this.delayTable.removeAllAckedMessageIds(removableIds);
        }
    }

    /**
     * Updates an delay table entry for given message and host
     *
     * @param m     The message
     * @param host  The host which contains a copy of this message
     * @param delay The estimated delay
     * @param time  The entry was last changed
     */
    public void updateDelayTableEntry(Message m, DTNHost host, double delay, double time) {
        DelayEntry delayEntry;

        if ((delayEntry = this.delayTable.getDelayEntryByMessageId(m.getId())) == null) {
            delayEntry = new DelayEntry(m);
            this.delayTable.addEntry(delayEntry);
        }
        assert ((delayEntry != null) && (this.delayTable.getDelayEntryByMessageId(m.getId()) != null));

        if (delayEntry.contains(host)) {
            delayEntry.setHostDelay(host, delay, time);
        } else {
            delayEntry.addHostDelay(host, delay, time);
        }
    }

    @Override
    public boolean createNewMessage(Message m) {
        boolean stat = super.createNewMessage(m);

        // if message was created successfully add the according delay table entry
        if (stat) {
            updateDelayTableEntry(m, getHost(), estimateDelay(m, getHost(), true), SimClock.getTime());
            // System.out.println("PESAN BARU "+m.getId()+" di buat oleh "+getHost()+"
            // ukuran "+m.getSize()+" Tujuan "+m.getTo());
        }

        return stat;
    }

    @Override
    public int receiveMessage(Message m, DTNHost from) {
        int stat = super.receiveMessage(m, from);
        // if message was received successfully add the according delay table entry
        if (stat == 0) {
            DTNHost host = getHost();
            double time = SimClock.getTime();
            double delay = estimateDelay(m, host, true);
            updateDelayTableEntry(m, host, delay, time);
            ((KnapsackDP) from.getRouter()).updateDelayTableEntry(m, host, delay, time);
        }

        return stat;
    }

    /**
     * Returns the current marginal utility (MU) value for a connection of two
     * nodes
     *
     * @param msg  One message of the message queue
     * @param con  The connection
     * @param host The host which contains a copy of this message
     * @return the current MU value
     */
    private double getMarginalUtility(Message msg, DTNHost otherHost, DTNHost host) {
        final KnapsackDP otherRouter = (KnapsackDP) (otherHost.getRouter());

        return getMarginalUtility(msg, otherRouter, host);
    }

    /**
     * Returns the current marginal utility (MU) value for a host of a specified
     * router
     *
     * @param msg    One message of the message queue
     * @param router The router which the message could be send to
     * @param host   The host which contains a copy of this message
     * @return the current MU value
     */
    private double getMarginalUtility(Message msg, KnapsackDP router, DTNHost host) {
        double marginalUtility = 0.0;
        double utility = 0.0; // U(i): The utility of the message (packet) i
        double utilityOld = 0.0;

        assert (this != router);
        utility = router.computeUtility(msg, host, true);
        utilityOld = this.computeUtility(msg, host, true);

        // s(i): The size of message i
        // delta(U(i)) / s(i)
        if ((utilityOld == -INFINITY) && (utility != -INFINITY)) {
            marginalUtility = (Math.abs(utility) / msg.getSize());
        } else if ((utility == -INFINITY) && (utilityOld != -INFINITY)) {
            marginalUtility = (Math.abs(utilityOld) / msg.getSize());
        } else if ((utility == utilityOld) && (utility != -INFINITY)) {
            marginalUtility = (Math.abs(utility) / msg.getSize());
        } else {
            marginalUtility = (utility - utilityOld) / msg.getSize();
        }

        return marginalUtility;
    }

    protected double computeUtility(Message msg, DTNHost host, boolean recompute) {
        double utility = 0.0; // U(i): The utility of the message (packet) i
        double packetDelay = 0.0; // D(i): The expected delay of message i

        switch (this.algorithm) {
            // minimizing average delay
            // U(i) = -D(i)
            case AVERAGE_DELAY: {
                packetDelay = estimateDelay(msg, host, recompute);
                utility = -packetDelay;
                break;
            }
            // minimizing missed deadlines
            // L(i) : The life time of message (packet) i
            // T(i) : The time since creation of message i
            // a(i) : A random variable that determines the remaining time to deliver
            // message i
            // P(a(i)): The probability that the message will be delivered within its
            // deadline
            //
            // / P(a(i) < L(i) - T(i)) , L(i) > T(i)
            // U(i) = <|
            // \ 0 , otherwise
            case MISSED_DEADLINES: {
                // life time in seconds
                double lifeTime = msg.getTtl() * 60;
                // creation time in seconds
                double timeSinceCreation = SimClock.getTime() - msg.getCreationTime();
                double remainingTime = computeRemainingTime(msg);
                if (lifeTime > timeSinceCreation) {
                    // compute remaining time by using metadata
                    if (remainingTime < lifeTime - timeSinceCreation) {
                        utility = lifeTime - timeSinceCreation - remainingTime;
                    } else {
                        utility = 0;
                    }
                } else {
                    utility = 0;
                }
                packetDelay = computePacketDelay(msg, host, remainingTime);
                break;
            }
            // minimizing maximum delay
            // S : Set of all message in buffer of X
            //
            // / -D(i) , D(i) >= D(j) for all j element of S
            // U(i) = <|
            // \ 0 , otherwise
            case MAXIMUM_DELAY: {
                packetDelay = estimateDelay(msg, host, recompute);
                Collection<Message> msgCollection = getMessageCollection();
                for (Message m : msgCollection) {
                    if (m.equals(msg)) {
                        continue; // skip
                    }
                    if (packetDelay < estimateDelay(m, host, recompute)) {
                        packetDelay = 0;
                        break;
                    }
                }
                utility = -packetDelay;
                break;
            }
            case RANDOM: {
                utility = java.util.concurrent.ThreadLocalRandom.current().nextDouble(0.0, 1.0);
                break;
            }
            case UNIFORM: {
                utility = 1.0;
                break;
            }
            case PROPHET: {
                utility = this.getPredFor(msg.getTo());
                break;
            }
            default:
        }
        return utility;
    }

    /*
     * Estimate-Delay Algorithm
     * 
     * 1) Sort packets in Q in decreasing order of T(i). Let b(i) be the sum of
     * sizes of packets that precede i, and B the expected transfer opportunity
     * in bytes between X and Z.
     * 
     * 2. X by itself requires b(i)/B meetings with Z to deliver i. Compute the
     * random variable MX(i) for the corresponding delay as
     * MX(i) = MXZ + MXZ + . . . b(i)/B times (4)
     * 
     * 3. Let X1 , . . . , Xk ⊇ X be the set of nodes possessing a replica of i.
     * Estimate remaining time a(i) as
     * a(i) = min(MX1(i), . . . , MXk(i))
     * 
     * 4. Expected delay D(i) = T(i) + E[a(i)]
     */
    /**
     * Returns the expected delay for a message if this message was sent to a
     * specified host
     *
     * @param msg The message which will be transfered
     * @return the expected packet delay
     */
    private double estimateDelay(Message msg, DTNHost host, boolean recompute) {
        double remainingTime = 0.0; // a(i): random variable that determines the remaining time to deliver message i
        double packetDelay = 0.0; // D(i): expected delay of message i

        // if delay table entry for this message doesn't exist or the delay table
        // has changed recompute the delay entry
        if ((recompute)
                && ((this.delayTable.delayHasChanged(msg.getId())) || (this.delayTable.getDelayEntryByMessageId(
                        msg.getId()).getDelayOf(host) == null) /*
                                                                * || (delayTable.getEntryByMessageId(msg.getId()).
                                                                * getDelayOf(host) == INFINITY))
                                                                */)) {
            // compute remaining time by using metadata
            remainingTime = computeRemainingTime(msg);
            packetDelay = Math.min(this.INFINITY, computePacketDelay(msg, host, remainingTime));

            // update delay table
            updateDelayTableEntry(msg, host, packetDelay, SimClock.getTime());
        } else {
            packetDelay = this.delayTable.getDelayEntryByMessageId(msg.getId()).getDelayOf(host);
        }

        return packetDelay; // Nilai perkiraan delay dari Messages
    }

    private double computeRemainingTime(Message msg) {
        double transferTime = this.INFINITY; // MX(i):random variable for corresponding transfer time delay
        double remainingTime = 0.0; // a(i): random variable that determines the remaining time to deliver message i

        remainingTime = computeTransferTime(msg, msg.getTo());
        if (this.delayTable.getDelayEntryByMessageId(msg.getId()) != null) {
            for (Map.Entry<DTNHost, Tuple<Double, Double>> entry : this.delayTable.getDelayEntryByMessageId(msg.getId())
                    .getDelays()) {
                DTNHost host = entry.getKey();
                if (host == getHost()) {
                    continue; // skip
                }
                transferTime = ((KnapsackDP) host.getRouter()).computeTransferTime(msg,
                        msg.getTo()); // MXm(i)
                // with
                // m
                // element
                // of
                // [0...k]
                remainingTime = Math.min(transferTime, remainingTime); // a(i) = min(MX0(i), MX1(i), ... ,MXk(i))
            }
        }

        return remainingTime; // Sisa waktu untuk mengirim Messages
    }

    private double computeTransferTime(Message msg, DTNHost host) {
        Collection<Message> msgCollection = getMessageCollection();
        double transferOpportunity = 0; // B: expected transfer opportunity in bytes between X and Z
        double packetsSize = 0.0; // b(i): sum of sizes of packets that precede the actual message
        double transferTime = this.INFINITY; // MX(i):random variable for corresponding transfer time delay
        double meetingTime = 0.0; // MXZ: random variable that represent the meeting Time between X and Y

        // packets are already sorted in decreasing order of receive time
        // sorting packets in decreasing order of time since creation is optional
        // sort packets in decreasing order of time since creation
        // compute sum of sizes of packets that precede the actual message
        for (Message m : msgCollection) {
            if (m.equals(msg)) {
                continue; // skip
            }
            packetsSize = packetsSize + m.getSize();
        }

        // compute transfer time
        transferOpportunity = this.delayTable.getAvgTransferOpportunity(); // B in bytes

        MeetingEntry entry = this.delayTable.getIndirectMeetingEntry(this.getHost().getAddress(), host.getAddress());
        // a meeting entry of null means that these hosts have never met -> set transfer
        // time to maximum
        if (entry == null) {
            transferTime = this.INFINITY; // MX(i)
        } else {
            meetingTime = entry.getAvgMeetingTime(); // MXZ
            transferTime = meetingTime * Math.ceil(packetsSize / transferOpportunity); // MX(i) = MXZ * ceil[b(i) / B]
        }

        return transferTime; // Waktu untuk transfer Messages
    }

    private double computePacketDelay(Message msg, DTNHost host, double remainingTime) {
        double timeSinceCreation = 0.0; // T(i): time since creation of message i
        double expectedRemainingTime = 0.0; // A(i): expected remaining time E[a(i)]=A(i)
        double packetDelay = 0.0; // D(i): expected delay of message i

        // TODO E[a(i)]=Erwartungswert von a(i) mit a(i)=remainingTime
        expectedRemainingTime = remainingTime;

        // compute packet delay
        timeSinceCreation = SimClock.getTime() - msg.getCreationTime();
        packetDelay = timeSinceCreation + expectedRemainingTime;

        return packetDelay; // Nilai delay sebuah Messages
    }

    @Override
    public void update() {
        // Cache velocity/speed and direction unit vector
        DTNHost thisHost = getHost();
        if (thisHost.getPath() != null) {
            lastKnownSpeed.put(thisHost, thisHost.getPath().getSpeed());
            Coord origin = thisHost.getLocation();
            Coord dest = getNextWaypoint(thisHost);
            double dx = dest.getX() - origin.getX();
            double dy = dest.getY() - origin.getY();
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 1e-9) {
                lastKnownDirection.put(thisHost, new double[]{ dx / dist, dy / dist });
            }
        }

        super.update();
        // Cek sedang transfer atau tidak
        if (isTransferring() || !canStartTransfer()) {
            return;
        }
        // Melakukan pertukaran pesan secara langsung
        // Try messages that can be delivered to final recipient
        if (exchangeDeliverableMessages() != null) {
            return;
        }
        // Otherwise do RAPID-style message exchange
        // (Mengoptimalkan pengiriman pesan di lingkungan dengan mobilitas tinggi dan
        // topologi jaringan yang dinamis)
        if (tryOtherMessages() != null) {
            return;
        }
    }

    @Override
    protected void transferDone(Connection con) {
        Message transferred = this.getMessage(con.getMessage().getId());
        if (String.valueOf(getHost().toString().charAt(0)).equals("s")) {
            this.deleteMessage(transferred.getId(), false);
        }
        // System.out.println(String.valueOf(getHost().toString().charAt(0)).equals("d"));
    }

    private Tuple<Tuple<Message, Connection>, Double> tryOtherMessages() {
        List<Tuple<Tuple<Message, Connection>, Double>> messages = new ArrayList<Tuple<Tuple<Message, Connection>, Double>>();

        for (Connection con : getConnections()) {
            DTNHost other = con.getOtherNode(getHost());
            KnapsackDP otherRouter = (KnapsackDP) other.getRouter();
            // Memeriksa apakah host tersebut sedang dalam proses mentransfer pesan atau
            // tidak
            if (otherRouter.isTransferring()) {
                continue; // skip hosts that are transferring
            }
            // Memeriksa apakah sensor atau tidak
            if (String.valueOf(other.toString().charAt(0)).equals("s")) {
                continue; // skip kalau sensor
            }

            if (String.valueOf(getHost().toString().charAt(0)).equals("s")) {
                for (Message m : getHost().getMessageCollection()) {
                    double u = getMarginalUtility(m, other, getHost());
                    Tuple<Message, Connection> t1 = new Tuple<Message, Connection>(m, con);
                    Tuple<Tuple<Message, Connection>, Double> t2 = new Tuple<Tuple<Message, Connection>, Double>(t1, u);
                    messages.add(t2);
                }
            } else {

                double mu = 0.0;
                for (Message m : this.tempMsgTerpilih) {
                    if (otherRouter.hasMessage(m.getId())) {
                        continue; // skip messages that the other one already has
                    }

                    mu = getMarginalUtility(m, other, getHost());

                    if ((mu) <= 0) {
                        continue; // skip messages with a marginal utility smaller or equals to 0.
                    }
                    Tuple<Message, Connection> t1 = new Tuple<Message, Connection>(m, con);
                    Tuple<Tuple<Message, Connection>, Double> t2 = new Tuple<Tuple<Message, Connection>, Double>(t1,
                            mu);
                    messages.add(t2);
                }
                // }
            }
        }

        this.delayTable.setChanged(false);
        if (messages == null) {
            return null;
        }
        // Mencoba mengirimkan pesan-pesan tersebut ke host penerima
        return tryTupleMessagesForConnected(messages); // try to send messages
        // return tryAllMessagesToAllConnections();
    }

    /**
     * Tries to send messages for the connections that are mentioned in the
     * Tuples in the order they are in the list until one of the connections
     * starts transferring or all tuples have been tried.
     *
     * @param tuples The tuples to try
     * @return The tuple whose connection accepted the message or null if none
     *         of the connections accepted the message that was meant for them.
     */
    // Mencoba untuk mengirimkan setiap pesan pada list pesan yang belum terkirim ke
    // setiap koneksi pada list koneksi yang tersedia
    private Tuple<Tuple<Message, Connection>, Double> tryTupleMessagesForConnected(
            List<Tuple<Tuple<Message, Connection>, Double>> tuples) {
        // Memeriksa apakah kosong atau tidak
        if (tuples.size() == 0) {
            return null;
        }

        for (Tuple<Tuple<Message, Connection>, Double> t : tuples) {
            Message m = (t.getKey()).getKey();
            Connection con = (t.getKey()).getValue();
            if (startTransfer(m, con) == RCV_OK) { // Proses transfer pesan telah dimulai, dan method akan mengembalikan
                                                   // tuple tersebut sebagai hasil
                return t;
            }
        }
        return null;
    }

    public DelayTable getDelayTable() {
        return this.delayTable;
    }

    @Override
    public KnapsackDP replicate() {
        return new KnapsackDP(this);

    }

    private enum UtilityAlgorithm {
        AVERAGE_DELAY,
        MISSED_DEADLINES,
        MAXIMUM_DELAY,
        RANDOM,
        UNIFORM,
        PROPHET;
    }

    public Map<Integer, DTNHost> getHostMapping() {
        return this.hostMapping;
    }

    public DTNHost getOtherHostt() {
        return this.getOtherHost;
    }

    public double getMeetingProb(DTNHost host) {
        MeetingEntry entry = this.delayTable.getIndirectMeetingEntry(getHost().getAddress(), host.getAddress());
        if (entry != null) {
            double prob = (entry.getAvgMeetingTime() * entry.getWeight()) / SimClock.getTime();
            return prob;
        }
        return 0.0;
    }

    public double getAvgDurations(DTNHost nodes) {
        List<Duration> list = getListDuration(nodes);
        Iterator<Duration> duration = list.iterator();
        double hasil = 0;
        // Durasi dihitung dengan cara mengambil selisih antara waktu akhir dan waktu
        // awal pada setiap objek Duration pada List
        // lalu dijumlahkan dan dibagi dengan jumlah objek Duration pada List
        while (duration.hasNext()) {
            Duration d = duration.next();
            hasil += (d.end - d.start);
        }
        int avgDuration = (int) (hasil / list.size());
        if (avgDuration == 0) { // Jika hasilnya 0, maka nilai rata-rata akan diatur menjadi 1
            avgDuration = 1;
        }
        return avgDuration;
    }

    public List<Duration> getListDuration(DTNHost nodes) {
        if (this.connHistory.containsKey(nodes)) {
            return this.connHistory.get(nodes);
        } else {
            List<Duration> d = new LinkedList<>();
            return d;
        }
    }

    public void cekSyaratKnapsackSend(DTNHost thisHost, DTNHost otherHost) {
        // System.out.println(getSyaratKnapsack(thisHost, otherHost));
        if (getSyaratKnapsack(thisHost, otherHost)) {
            getUtilityMsgToArrForSend(thisHost, otherHost); // mendapatkan nilai utilitas setiap pesan
            knapsackSend(thisHost, otherHost); // mengirim pesan ke node tujuan menggunakan algoritma Knapsack
        } else {
            // System.out.println("Msg :"+getMessageCollection().toString());
            // System.out.println("Msg terpilih : " + this.tempMsgTerpilih.toString());
            this.tempMsgTerpilih.addAll(getMessageCollection());
        }
    }

    public boolean getSyaratKnapsack(DTNHost thisHost, DTNHost otherHost) {
        int retriction = getRetrictionForSend(thisHost, otherHost);
        // System.out.println("Retriction " + thisHost + ":" + retriction);
        int isiBuffer = ((thisHost.getRouter().getBufferSize() - thisHost.getRouter().getFreeBufferSize()) / bytes);
        // System.out.println("isiBuffer :" + isiBuffer);
        return isiBuffer > retriction;
    }

    public int getRetrictionForSend(DTNHost thisHost, DTNHost otherHost) {
        return retrictionPerPeer.get(otherHost);
    }

    public int getTransferSpeed(DTNHost thisHost) {
        return thisHost.getInterfaces().get(0).getTransmitSpeed();
    }

    public void getUtilityMsgToArrForSend(DTNHost thisHost, DTNHost otherHost) {
        this.lengthMsg.clear();
        this.utilityMsg.clear();
        for (Message m : thisHost.getMessageCollection()) {
            this.lengthMsg.add(m.getSize() / bytes); // in kiloBytes

            double util = 0;
            if (this.algorithm == UtilityAlgorithm.PROPHET) {
                util = ((KnapsackDP) otherHost.getRouter()).getPredFor(m.getTo());
            } else {
                util = computeUtility(m, thisHost, true);
            }
            this.utilityMsg.add(util);
        }
    }

    /**
     * Melakukan proses knapsack untuk menentukan pesan mana yang akan dikirim saat
     * melakukan transfer data.
     * Metode ini mengimplementasikan algoritma knapsack 0/1 dengan pendekatan
     * pemrograman dinamis.
     * Pesan-pesan yang memiliki utilitas tinggi akan dipilih untuk dikirimkan
     * berdasarkan kapasitas knapsack yang tersedia.
     *
     * @param thisHost  Host pengirim
     * @param otherHost Host penerima
     */

    // Perubahan Terbaru
    public void knapsackSend(DTNHost thisHost, DTNHost otherHost) {
        this.tempMsg.clear();
        this.tempMsg.addAll(this.getMessageCollection());
        int jumlahMsg = 0;
        int retriction = 0;
        jumlahMsg = this.tempMsg.size();
        retriction = this.getRetrictionForSend(thisHost, otherHost);
        double[][] bestSolutionSend = new double[jumlahMsg + 1][retriction + 1];

        // --- TRANSFORMASI KNAPSACK NEGATIF KE POSITIF ---
        // Karena U(i) = -D(i), nilainya selalu negatif.
        // Kita butuh mencari D_max (delay maksimum = nilai utilitas Paling Negatif)
        double maxDelayNegative = 0;
        for (int m = 0; m < this.utilityMsg.size(); m++) {
            if (this.utilityMsg.get(m) < maxDelayNegative) {
                maxDelayNegative = this.utilityMsg.get(m);
            }
        }

        // maxDelayNegative adalah sebuah angka negatif, misalnya -20.
        // Jika kita kurangi setiap utilitas pesan dengan nilai ini: U(i) -
        // maxDelayNegative
        // Misalnya U(i) = -7, maka (-7) - (-20) = 13 (positif, makin kecil delay makin
        // besar profit)
        // Profit ini yang kita masukkan ke Knapsack DP.

        // Inisialisasi tabel solusi dengan nilai 0
        for (int i = 0; i <= jumlahMsg; i++) {
            for (int length = 0; length <= retriction; length++) {
                if (i == 0 || length == 0) {
                    bestSolutionSend[i][length] = 0;
                } else if (this.lengthMsg.get(i - 1) <= length) {
                    // Kalkulasi Profit berdasarkan pergeseran nilai D_max
                    double transformedProfit = this.utilityMsg.get(i - 1) - maxDelayNegative;

                    // Jika panjang pesan lebih kecil atau sama dengan kapasitas knapsack yang
                    // tersisa, pilih pesan dengan utilitas maksimum
                    bestSolutionSend[i][length] = Math.max(bestSolutionSend[i - 1][length],
                            transformedProfit + bestSolutionSend[i - 1][length - this.lengthMsg.get(i - 1)]);
                } else {
                    // Jika panjang pesan lebih besar dari kapasitas knapsack yang tersisa, lewati
                    // pesan ini
                    bestSolutionSend[i][length] = bestSolutionSend[i - 1][length];
                }
            }
        }
        // Mengumpulkan pesan-pesan yang akan dikirim berdasarkan solusi terbaik
        int temp = retriction;
        for (int j = jumlahMsg; j >= 1; j--) {
            if (bestSolutionSend[j][temp] > bestSolutionSend[j - 1][temp]) {
                this.tempMsgTerpilih.add(this.tempMsg.get(j - 1));
                temp = temp - this.lengthMsg.get(j - 1);
            }
        }
    }

    /**
     * Override makeRoomForMessage untuk mensimulasikan asumsi Unlimited Storage.
     * Tidak membuang pesan apapun karena memori dianggap tidak terbatas,
     * tetapi tetap mengembalikan nilai true agar router dapat menerima pesan baru.
     */
    @Override
    protected boolean makeRoomForMessage(int size) {
        return true; 
    }

    // --- PROPHET HELPER METHODS ---
    private void initProphet(Settings s) {
        if (this.algorithm == UtilityAlgorithm.PROPHET) {
            Settings prophetSettings = new Settings(PROPHET_NS);
            secondsInTimeUnit = prophetSettings.getInt(SECONDS_IN_UNIT_S);
            if (prophetSettings.contains(BETA_S)) {
                beta = prophetSettings.getDouble(BETA_S);
            } else {
                beta = DEFAULT_BETA;
            }
            initPreds();
        }
    }

    private void initPreds() {
        this.preds = new HashMap<DTNHost, Double>();
    }

    public double getPredFor(DTNHost host) {
        ageDeliveryPreds(); // make sure preds are updated before getting
        if (preds.containsKey(host)) {
            return preds.get(host);
        } else {
            return 0.0;
        }
    }

    private void updateDeliveryPredFor(DTNHost host) {
        double oldValue = getPredFor(host);
        double newValue = oldValue + (1 - oldValue) * P_INIT;
        preds.put(host, newValue);
    }

    private void updateTransitivePreds(DTNHost host) {
        MessageRouter otherRouter = host.getRouter();
        assert otherRouter instanceof KnapsackDP : "PROPHET mode only works with other KnapsackDP routers";
        
        double pForHost = getPredFor(host); // P(a,b)
        Map<DTNHost, Double> othersPreds = ((KnapsackDP) otherRouter).getDeliveryPreds();
        
        for (Map.Entry<DTNHost, Double> e : othersPreds.entrySet()) {
            if (e.getKey() == getHost()) {
                continue; // don't add yourself
            }
            
            double pOld = getPredFor(e.getKey()); // P(a,c)_old
            double pNew = pOld + (1 - pOld) * pForHost * e.getValue() * beta;
            preds.put(e.getKey(), pNew);
        }
    }

    private void ageDeliveryPreds() {
        if (secondsInTimeUnit == 0) return; // Guard to avoid division by zero
        double timeDiff = (SimClock.getTime() - this.lastAgeUpdate) / secondsInTimeUnit;
        if (timeDiff == 0) {
            return;
        }
        
        double mult = Math.pow(GAMMA, timeDiff);
        for (Map.Entry<DTNHost, Double> e : preds.entrySet()) {
            e.setValue(e.getValue() * mult);
        }
        
        this.lastAgeUpdate = SimClock.getTime();
    }

    private Map<DTNHost, Double> getDeliveryPreds() {
        ageDeliveryPreds(); // make sure the aging is done
        return this.preds;
    }
}