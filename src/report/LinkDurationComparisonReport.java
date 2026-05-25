package report;

import core.ConnectionListener;
import core.DTNHost;
import core.SimClock;

import java.util.HashMap;
import java.util.Map;

import routing.DecisionEngineRouter;
import routing.LinkDurationEstimationRouter;
import routing.RoutingDecisionEngine;

/**
 * Report:
 * Compare estimated link duration vs real link duration
 *
 * FIXED VERSION:
 * - Uses REAL velocity cache
 * - Compatible with fixed LinkDurationEstimationRouter
 * - No speed cache
 * - No direction cache
 * - No waypoint dependency
 */
public class LinkDurationComparisonReport extends Report
        implements ConnectionListener {

    /**
     * Connection start times
     */
    private Map<String, Double> startTimes;

    /**
     * Estimated durations
     */
    private Map<String, Double> estimatedDurations;

    public LinkDurationComparisonReport() {

        init();

        startTimes = new HashMap<>();

        estimatedDurations = new HashMap<>();

        write("NodePair\tEstimatedDuration\tRealDuration");
    }

    /**
     * Create unique pair key
     */
    private String getKey(DTNHost h1, DTNHost h2) {

        if (h1.getAddress() < h2.getAddress()) {
            return h1 + "<->" + h2;
        }

        return h2 + "<->" + h1;
    }

    @Override
    public void hostsConnected(DTNHost host1, DTNHost host2) {

        String key = getKey(host1, host2);

        // Store connection start time
        startTimes.put(key, SimClock.getTime());

        try {

            RoutingDecisionEngine de =
                    ((DecisionEngineRouter) host1.getRouter())
                            .getDecisionEngine();

            if (de instanceof LinkDurationEstimationRouter lder) {

                // =========================================================
                // FIX:
                // Use REAL velocity cache from delta-position estimation
                // =========================================================

                double estimated =
                        LinkDurationEstimationRouter
                                .estimateLinkDuration(
                                        host1,
                                        host2,
                                        lder.getDetectionRange(),
                                        lder.getLastKnownSpeed(),
                                        lder.getLastKnownDirection()
                                );

                estimatedDurations.put(key, estimated);

//                System.out.printf(
//                        "[REPORT] %s Estimated Duration = %.6f%n",
//                        key,
//                        estimated
//                );
            }

        } catch (Exception e) {

            System.out.println(
                    "[REPORT ERROR] Failed estimating link duration for "
                            + key
            );

            e.printStackTrace();

            estimatedDurations.put(key, -1.0);
        }
    }

    @Override
    public void hostsDisconnected(DTNHost host1, DTNHost host2) {

        String key = getKey(host1, host2);

        if (!startTimes.containsKey(key)) {
            return;
        }

        double start = startTimes.get(key);

        double end = SimClock.getTime();

        // =============================================================
        // REAL measured duration
        // =============================================================

        double realDuration = end - start;

        // =============================================================
        // Estimated duration
        // =============================================================

        double estimated =
                estimatedDurations.getOrDefault(key, -1.0);

        // =============================================================
        // Write report
        // =============================================================

        write(
                key + "\t"
                        + estimated + "\t"
                        + realDuration
        );

        System.out.printf(
                "[REPORT] %s Estimated=%.6f Real=%.6f%n",
                key,
                estimated,
                realDuration
        );

        // Cleanup
        startTimes.remove(key);

        estimatedDurations.remove(key);
    }

    @Override
    public void done() {

        write("");
        write("===== END OF LINK DURATION COMPARISON REPORT =====");

        super.done();
    }
}
