package steiner.algo;

import com.mxgraph.layout.mxIGraphLayout;
import com.mxgraph.layout.mxOrganicLayout;
import com.mxgraph.swing.mxGraphComponent;
import org.apache.commons.math3.linear.*;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import org.jgrapht.alg.spanning.PrimMinimumSpanningTree;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;
import com.aparapi.*;

import java.util.*;

import org.jgrapht.ext.*;
import org.jgrapht.Graph;

import javax.swing.*;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


public class Steiner_Global {
    public SteinerGraph getGraph() {
        return graph;
    }

    private SteinerGraph graph;
    private Set<Vertex> sourceSet;
    public Steiner_Global(SteinerGraph graph){
        this.graph = graph;
    }

    public Steiner_Global(SteinerGraph graph,double delta,double rho,double edgeThreshold){
        this.graph = graph;
        this.delta = delta;
        this.rho = rho;
        this.edgeThreshold = edgeThreshold;

        sourceSet  = new HashSet<>();
        for(Vertex vertex:this.graph.getGraph().vertexSet()){
            if(vertex.isSource()) {
                sourceSet.add(vertex);
                graph.getSourceList().add(vertex.getName());
            }
        }
    }

    private double I_0 = 100;   //todo
    private double delta;
    private double rho;
    private double edgeThreshold;



    public Set<Vertex> getSourceSet(){
        return sourceSet;
    }

    public void initial_approx(){
        for(Vertex sink: sourceSet){
            int n = graph.getGraph().vertexSet().size();
            double[] pressure = new double[n];
            boolean[] visited = new boolean[n];
            Queue<Vertex> queue = new LinkedList<>();

            queue.add(sink);
            visited[sink.getName()] = true;
            int level = 0;

            while(!queue.isEmpty()){
                int size = queue.size();
                while(size-- > 0){
                    Vertex curr = queue.poll();
                    visited[curr.getName()] = true;
                    pressure[curr.getName()] = level;
                    for(Vertex neighbor: curr.getNeighbor(graph.getGraph())){
                        if(!visited[neighbor.getName()])
                            queue.add(neighbor);
                    }
                }
                level += 50;
            }

            graph.getPressure().add(pressure);
            //System.out.println("Done calculating pressure with " + sink.getName() + " as sink.");
        }
        System.out.println("Initialization completed.");
    }

    public void initial(){
        double[][] adjmatrix = this.graph.getAdjmatrix();

        for(Vertex sink:sourceSet){   //Take one source as sink each loop
            double[][] coefficients = new double[graph.getGraph().vertexSet().size()][graph.getGraph().vertexSet().size()];
            double[] constant = new double[graph.getGraph().vertexSet().size()];
            for(Vertex curr_vertex:this.graph.getGraph().vertexSet()){  //get the constant done
                if(curr_vertex.isSource()){     //is source
                    if(curr_vertex.getName()==sink.getName()){  //is source & sink
                        constant[curr_vertex.getName()] = -(graph.getSourceNum()-1) * I_0;
                    }else{  //is source but not sink
                        constant[curr_vertex.getName()] = I_0;
                    }
                }else{  //not source
                    constant[curr_vertex.getName()] = 0;
                }
            }

            for(Vertex curr_vertex:this.graph.getGraph().vertexSet()){  //get the coefficients done
                double P_i_coefficient = 0;
                for(Vertex neighbor:curr_vertex.getNeighbor(graph.getGraph())){ //for each neighbor
                    int curr_index = curr_vertex.getName();
                    int neighbor_index = neighbor.getName();
                    double conductivity = graph.getGraph().getEdge(curr_vertex,neighbor).getConductivity();
                    P_i_coefficient += (conductivity / adjmatrix[curr_index][neighbor_index]);
                    coefficients[curr_index][neighbor_index] = - conductivity / adjmatrix[curr_index][neighbor_index];
                }
                if(curr_vertex.getName()!=sink.getName())
                    coefficients[curr_vertex.getName()][curr_vertex.getName()] = P_i_coefficient;
            }

            RealMatrix matrix = new Array2DRowRealMatrix(coefficients,false);
            DecompositionSolver solver = new QRDecomposition(matrix).getSolver();
            RealVector vector = new ArrayRealVector(constant);
            graph.getPressure().add(solver.solve(vector).toArray());
            //System.out.println("Done calculating pressure with " + sink.getName() + " as sink.");
        }
        System.out.println("Initialization completed.");
    }
    public void iteration_ksub_gpu(int T_c, final Vertex sink,int outerloop, final ArrayList<double[]> prevPressure, ExecutorService threadPool){
        final double[][] adjmatrix = graph.getAdjmatrix();
        for(int i=0;i<T_c;i++){
            Edge[] edgeArray = graph.getGraph().edgeSet().toArray(new Edge[graph.getGraph().edgeSet().size()]);
            final int size = edgeArray.length;
            final double[] pressure = graph.getPressure().get(graph.getSourceList().indexOf(sink.getName()));
            final double[] conductivityArray = new double[edgeArray.length];
            final double[] weightArray = new double[edgeArray.length];
            final int[] targetIndex = new int[edgeArray.length];
            final int[] sourceIndex = new int[edgeArray.length];
            for(int j=0;j<edgeArray.length;j++) {
                conductivityArray[j] = edgeArray[j].getConductivity();
                weightArray[j] = edgeArray[j].getWeight();
                targetIndex[j] = ((Vertex)edgeArray[j].getTarget()).getName();
                sourceIndex[j] = ((Vertex)edgeArray[j].getSource()).getName();
            }
            final double[][] sourcesPressures = prevPressure.toArray(new double[0][0]);

            Kernel edgeKernel = new Kernel() {
                @Override
                public void run() {
                    int gid = getGlobalId();
                    double Kij = 0;
                    for(int i=0; i<sourcesPressures.length; i++){
                        Kij += Math.abs(sourcesPressures[i][targetIndex[gid]] - sourcesPressures[i][sourceIndex[gid]]);
                    }
                    Kij = 1 + 0.0005 * Kij / weightArray[gid] - 0.001 * weightArray[gid];
                    double new_conductivity = Kij * conductivityArray[gid];
                    conductivityArray[gid] = new_conductivity;
                }
            };
            edgeKernel.execute(size);
            edgeKernel.dispose();

            for(int j=0;j<edgeArray.length;j++){
                edgeArray[j].setConductivity(conductivityArray[j]);
            }
/*
            for(final Edge edge:graph.getGraph().edgeSet()){  //Eq6
                Vertex target = (Vertex)edge.getTarget();
                Vertex source = (Vertex)edge.getSource();
                double Kij = 0;
                for(Vertex s: sourceSet){
                    final double[] sourcePressure = prevPressure.get(graph.getSourceList().indexOf(s.getName()));
                    Kij += Math.abs(sourcePressure[target.getName()] - sourcePressure[source.getName()]);
                }

                //double temp1 = delta * Kij / edge.getWeight();
                //double temp2 = rho * edge.getWeight();
                //System .out .println(temp1 - temp2); //Check the difference of the 2 factors

                Kij = 1 + delta * Kij / edge.getWeight() - rho * edge.getWeight();
                double new_conductivity = Kij * edge.getConductivity();
                edge.setConductivity(new_conductivity);
            }
*/
            if(outerloop%10 == 0)
                this.cutEdgeProportional( 0.28 * outerloop/10);

            final CountDownLatch vertexLatch = new CountDownLatch(graph.getGraph().vertexSet().size());

            for(final Vertex vertex:graph.getGraph().vertexSet()){
                threadPool.submit(new Runnable() {
                    @Override
                    public void run() {
                        try{
                            double sum_above = 0;
                            double sum_below = 0.000001;
                            for(Vertex neighbor:vertex.getNeighbor(graph.getGraph())){
                                double conductivity = graph.getGraph().getEdge(vertex,neighbor).getConductivity();
                                sum_below = sum_below + conductivity / adjmatrix[vertex.getName()][neighbor.getName()];
                                sum_above = sum_above + conductivity * pressure[neighbor.getName()] / adjmatrix[vertex.getName()][neighbor.getName()];
                            }

                            if(vertex.isSource()){
                                if(vertex.getName()==sink.getName())
                                    sum_above = 0;
                                else
                                    sum_above = sum_above + I_0;
                            }else
                                ;

                            graph.getPressure().get(graph.getSourceList().indexOf(sink.getName()))[vertex.getName()] = sum_above/sum_below;
                        } finally {
                            vertexLatch.countDown();
                        }
                    }
                });
            }

            try {
                vertexLatch.await();
            } catch (Exception e){
                e.printStackTrace();
            }
        }
    }

    public void iteration_ksub(int T_c, final Vertex sink,int outerloop, final ArrayList<double[]> prevPressure, ExecutorService threadPool){
        final double[][] adjmatrix = graph.getAdjmatrix();
        for(int i=0;i<T_c;i++){
            final double[] pressure = graph.getPressure().get(graph.getSourceList().indexOf(sink.getName()));
            final CountDownLatch edgeLatch = new CountDownLatch(graph.getGraph().edgeSet().size());
            for(final Edge edge:graph.getGraph().edgeSet()){  //Eq6
                threadPool.submit(new Runnable() {
                    @Override
                    public void run() {
                        try{
                            Vertex target = (Vertex)edge.getTarget();
                            Vertex source = (Vertex)edge.getSource();
                            double Kij = 0;
                            for(Vertex s: sourceSet){
                                final double[] sourcePressure = prevPressure.get(graph.getSourceList().indexOf(s.getName()));
                                Kij += Math.abs(sourcePressure[target.getName()] - sourcePressure[source.getName()]);
                            }

                            //double temp1 = delta * Kij / edge.getWeight();
                            //double temp2 = rho * edge.getWeight();
                            //System .out .println(temp1 - temp2); //Check the difference of the 2 factors

                            Kij = 1 + delta * Kij / edge.getWeight() - rho * edge.getWeight();
                            double new_conductivity = Kij * edge.getConductivity();
                            edge.setConductivity(new_conductivity);
                        }finally {
                            edgeLatch.countDown();
                        }
                    }
                });
            }
            try {
                edgeLatch.await();
            } catch (Exception e){
                e.printStackTrace();
            }

            if(outerloop%10 == 0)
                this.cutEdgeProportional( edgeThreshold * outerloop/10);

            final CountDownLatch vertexLatch = new CountDownLatch(graph.getGraph().vertexSet().size());

            for(final Vertex vertex:graph.getGraph().vertexSet()){
                threadPool.submit(new Runnable() {
                    @Override
                    public void run() {
                        try{
                            double sum_above = 0;
                            double sum_below = 0.000001;
                            for(Vertex neighbor:vertex.getNeighbor(graph.getGraph())){
                                double conductivity = graph.getGraph().getEdge(vertex,neighbor).getConductivity();
                                sum_below = sum_below + conductivity / adjmatrix[vertex.getName()][neighbor.getName()];
                                sum_above = sum_above + conductivity * pressure[neighbor.getName()] / adjmatrix[vertex.getName()][neighbor.getName()];
                            }

                            if(vertex.isSource()){
                                if(vertex.getName()==sink.getName())
                                    sum_above = 0;
                                else
                                    sum_above = sum_above + I_0;
                            }else
                                ;

                            graph.getPressure().get(graph.getSourceList().indexOf(sink.getName()))[vertex.getName()] = sum_above/sum_below;
                        } finally {
                            vertexLatch.countDown();
                        }
                    }
                });
            }

            try {
                vertexLatch.await();
            } catch (Exception e){
                e.printStackTrace();
            }
        }
    }

    public void iteration_ksub_changed(int T_c){
        ExecutorService fixedThreadPool = Executors.newFixedThreadPool(8);
        for(int i=1;i<=T_c;i++) {
            final ArrayList<double[]> prevPressure = (ArrayList<double[]>) graph.getPressure().clone();
            for (Vertex sink : sourceSet) {
                iteration_ksub(1, sink, i, prevPressure, fixedThreadPool);
            }
            //System.out.println("Current round: " + i);
        }
        fixedThreadPool.shutdown();


        Graph<Vertex,Edge> g = graph.getGraph();

        System.out.println("Edge Sum before MST: " + graph.getEdgeWeightSum());

        PrimMinimumSpanningTree<Vertex, Edge> mst = new PrimMinimumSpanningTree<>(g);
        Set<Edge> MSTedgeSet = mst.getSpanningTree().getEdges();
        Collection<Edge> edgesToRemove = new HashSet<>();
        for(Edge e: g.edgeSet()){
            if(MSTedgeSet.contains(e))
                continue;
            else
                edgesToRemove.add(e);
        }
        g.removeAllEdges(edgesToRemove);

        for(int i=0; i<5; i++) {    // The loop time need to be determined manually now.
            Collection<Vertex> vertexToRemove = new HashSet<>();
            Iterator<Vertex> iterator_vertex = g.vertexSet().iterator();
            while (iterator_vertex.hasNext()) {
                Vertex curr_vertex = iterator_vertex.next();
                if (curr_vertex.getNeighbor(g).size() == 0 || (curr_vertex.getNeighbor(g).size() == 1 && !curr_vertex.isSource())) {
                    vertexToRemove.add(curr_vertex);
                }
            }
            g.removeAllVertices(vertexToRemove);
            if(vertexToRemove.size() == 0)
                break;
        }


        double sum = graph.getEdgeWeightSum();

        Iterator<Vertex> vertexIterator = this.getGraph().getGraph().vertexSet().iterator();
        int count = 0;
        while (vertexIterator.hasNext()) {
            Vertex curr_vertex = vertexIterator.next();
            if (curr_vertex.isSource())
                count++;
        }


        if (count == sourceSet.size()) {
            ConnectivityInspector<Vertex, Edge> inspector = new ConnectivityInspector<>(graph.getGraph());
            System.out.println(inspector.isConnected() + " \nEdge Sum:" + sum);
            System.out.println();
        } else {
            System.out.println("Fail to find the Steiner Tree, " + count + " sources in total.");
            System.out.println();
        }
    }

    public void visualization(){
        JFrame frame = new JFrame("Visualization");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Graph<Vertex,Edge> g = graph.getGraph();

        JGraphXAdapter<Vertex,Edge> graphXAdapter = new JGraphXAdapter<>(g);

        mxIGraphLayout layout = new mxOrganicLayout(graphXAdapter);
        layout.execute(graphXAdapter.getDefaultParent());

        frame.add(new mxGraphComponent(graphXAdapter));

        frame.pack();
        frame.setLocationByPlatform(true);
        frame.setVisible(true);
    }

    public void cutEdgeProportional(double edge_threshold){
        List<Edge> edges = new ArrayList<>(graph.getGraph().edgeSet());
        Collection<Edge> edgesToRemove = new HashSet<>();
        for(Edge e:edges){
            if(e.getConductivity()<edge_threshold) //abs
                edgesToRemove.add(e);
            //if(e.getConductivity()/source_sum<edge_threshold||e.getConductivity()/target_sum<edge_threshold)    //tweak here
            //   edgesToRemove.add(e);
        }
        graph.getGraph().removeAllEdges(edgesToRemove);

        Iterator<Vertex> iterator_vertex = graph.getGraph().vertexSet().iterator();
        Collection<Vertex> vertexToRemove = new HashSet<>();
        while(iterator_vertex.hasNext()){
            Vertex curr_vertex = iterator_vertex.next();
            if(curr_vertex.getNeighbor(graph.getGraph()).size() == 0||(curr_vertex.getNeighbor(graph.getGraph()).size()==1&&!curr_vertex.isSource())){
                vertexToRemove.add(curr_vertex);
            }
        }
        graph.getGraph().removeAllVertices(vertexToRemove);
    }
}
