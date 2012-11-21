/* This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public License
 as published by the Free Software Foundation, either version 3 of
 the License, or (props, at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

package org.opentripplanner.routing.algorithm.strategies;

import java.util.Arrays;

import org.opentripplanner.common.geometry.DistanceLibrary;
import org.opentripplanner.common.geometry.SphericalDistanceLibrary;
import org.opentripplanner.common.pqueue.BinHeap;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.graph.AbstractVertex;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.location.StreetLocation;
import org.opentripplanner.routing.vertextype.StreetVertex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class BidirectionalRemainingWeightHeuristic implements 
    RemainingWeightHeuristic, RemainingTimeHeuristic {

    private static final long serialVersionUID = 20111002L;

    private static Logger LOG = LoggerFactory.getLogger(LBGRemainingWeightHeuristic.class);

    Vertex target;

    double cutoff;

    double maxFound = 0;

    double[] weights;

    Graph g;

    private TransitLocalStreetService localStreetService;

    private DistanceLibrary distanceLibrary = SphericalDistanceLibrary.getInstance();

    public static int HEAD_START_MSEC = 100;
    
    /**
     * RemainingWeightHeuristic interface
     */

    public BidirectionalRemainingWeightHeuristic(Graph graph) {
        this.g = graph;
        this.localStreetService = g.getService(TransitLocalStreetService.class);
    }

    @Override
    public double computeInitialWeight(State s, Vertex target) {
        if (target == this.target && s.getOptions().maxWeight <= this.cutoff) {
            LOG.debug("reusing existing heuristic");
        } else {
            LOG.debug("spawning heuristic computation thread");
            this.target = target;
            new Thread(new Worker(s)).start();
//            try {
//                Thread.sleep(HEAD_START_MSEC);
//            } catch (InterruptedException e) {
//                throw new RuntimeException("Langauge specification error: checked exceptions");
//            }
//            LOG.debug("the heuristic has a head start. now we start searching.");
        }
        return 0;
    }

    @Override
    public double computeForwardWeight(State s, Vertex target) {
        return computeReverseWeight(s, target);
    }

    @Override
    public double computeReverseWeight(State s, Vertex target) {
        final Vertex v = s.getVertex();
        if (v instanceof StreetLocation)
            return 0;
        if (s.getWeight() < 10 * 60)
            return 0;
        int index = v.getIndex();
        if (index < weights.length) {
            double h = weights[index];
            return Double.isInfinite(h) ? maxFound : h;
        } else
            return 0;
    }

    @Override
    public void reset() {
    }

    /**
     * RemainingTimeHeuristic interface
     */
    @Override
    public void timeInitialize(State s, Vertex target) {
        throw new UnsupportedOperationException();
    }

    @Override
    public double timeLowerBound(State s) {
        return computeReverseWeight(s, null);
    }

    private /* inner */ class Worker implements Runnable {

        Vertex origin;
        
        RoutingRequest options;
        
        Worker (State s) {
            this.options = s.getOptions();
            this.origin = s.getVertex();
        }
        
        @Override
        public void run() {
            LOG.debug("recalc");
            cutoff = options.maxWeight;
            int nVertices = AbstractVertex.getMaxIndex();
            weights = new double[nVertices];
            Arrays.fill(weights, Double.POSITIVE_INFINITY);
            BinHeap<Vertex> q = new BinHeap<Vertex>();
            long t0 = System.currentTimeMillis();
            if (target instanceof StreetLocation) {
                for (Edge de : ((StreetLocation) target).getExtra()) {
                    Vertex gv;
                    if (options.isArriveBy()) {
                        gv = de.getToVertex();
                    } else {
                        gv = de.getFromVertex();
                    }
                    int gvi = gv.getIndex();
                    if (gv == target)
                        continue;
                    if (gvi >= nVertices)
                        continue;
                    weights[gvi] = 0;
                    q.insert(gv, 0);
                }
            } else {
                int i = target.getIndex();
                weights[i] = 0;
                q.insert(target, 0);
            }
            while (!q.empty()) {
                double uw = q.peek_min_key();
                Vertex u = q.extract_min();
                maxFound = uw;
                if (uw > cutoff)
                    break;
                if (u == origin) { // searching backward from target to origin
                    LOG.debug("hit origin.");
                    //break;
                }
                int ui = u.getIndex();
                if (uw > weights[ui])
                    continue;
                Iterable<Edge> edges;
                if (options.isArriveBy())
                    edges = u.getOutgoing();
                else
                    edges = u.getIncoming();
                for (Edge e : edges) {
                    Vertex v = options.isArriveBy() ? 
                        e.getToVertex() : e.getFromVertex();
                    double ew = e.weightLowerBound(options);
                    if (ew < 0) {
                        LOG.error("negative edge weight {} qt {}", ew, e);
                        continue;
                    }
                    double vw = uw + ew;
                    int vi = v.getIndex();
                    if (weights[vi] > vw) {
                        weights[vi] = vw;
                        // selectively rekeying did not seem to offer any speed advantage
                        q.insert(v, vw);
                        // System.out.println("Insert " + v + " weight " + vw);
                    }
                }
            }            
            LOG.info("End SSSP ({} msec)", System.currentTimeMillis() - t0);
            
        }
        
    }
    
}
