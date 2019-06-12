package steiner.algo;

import com.mxgraph.layout.mxIGraphLayout;
import com.mxgraph.layout.mxOrganicLayout;
import com.mxgraph.swing.mxGraphComponent;
import org.apache.commons.math3.linear.*;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;
import java.util.*;

import org.jgrapht.ext.*;
import org.jgrapht.Graph;

import javax.swing.*;
import java.util.List;


public class Steiner_Global {
    public SteinerGraph getGraph() {
        return graph;
    }

    private double prev_edge_sum;
    private SteinerGraph graph;
    public SortedMap<Integer,Double> ans;
    public Steiner_Global(SteinerGraph graph){
        this.graph = graph;
    }

    public Steiner_Global(SteinerGraph graph,double delta,double rho,double edge_threshold){
        this.graph = graph;
        this.delta = delta;
        this.rho = rho;
        this.edge_threshold = edge_threshold;
        ans = new TreeMap<>();

        sourceSet  = new HashSet<>();
        for(Vertex vertex:this.graph.getGraph().vertexSet()){
            if(vertex.isSource()) {
                sourceSet.add(vertex);
                graph.getSource_list().add(vertex.getName());
            }
        }
    }

    private double I_0 = 100;   //todo
    private double delta = 1.3;
    private double rho = 0.9;
    private double edge_threshold = 0.000000004;


    private Set<Vertex> sourceSet;

    public Set<Vertex> getSourceSet(){
        return sourceSet;
    }

    public void initial(){
        double[][] adjmatrix = this.graph.getAdjmatrix();
        sourceSet  = new HashSet<>();
        for(Vertex vertex:this.graph.getGraph().vertexSet()){
            if(vertex.isSource()) {
                sourceSet.add(vertex);
                graph.getSource_list().add(vertex.getName());
            }
        }

        for(Vertex sink:sourceSet){   //Take one source as sink each loop
            double[][] coefficients = new double[graph.getGraph().vertexSet().size()][graph.getGraph().vertexSet().size()];
            double[] constant = new double[graph.getGraph().vertexSet().size()];
            for(Vertex curr_vertex:this.graph.getGraph().vertexSet()){  //get the constant done
                if(curr_vertex.isSource()){     //is source
                    if(curr_vertex.getName()==sink.getName()){  //is source & sink
                        constant[curr_vertex.getName()] = -(graph.getSource_num()-1) * I_0;
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
            System.out.println("Done calculating pressure with " + sink.getName() + " as sink.");
        }
    }

    public double iteration_ksub(int T_c,Vertex sink,int outerloop){
        double[][] adjmatrix = graph.getAdjmatrix();
        for(int i=0;i<T_c;i++){
            double[] pressure = graph.getPressure().get(graph.getSource_list().indexOf(sink.getName()));
            for(Edge edge:graph.getGraph().edgeSet()){  //Eq6
                Vertex target = (Vertex)edge.getTarget();
                Vertex source = (Vertex)edge.getSource();
                double flux = Math.abs(edge.getConductivity()*(pressure[target.getName()]-pressure[source.getName()])/edge.getWeight());
                double new_conductivity = (Math.pow(flux,delta)/(I_0*I_0/adjmatrix.length*adjmatrix.length+Math.pow(flux,delta))) + rho * edge.getConductivity();
                edge.setConductivity(new_conductivity);
            }

            prev_edge_sum = graph.getEdgeWeightSum();
            this.cutEdgeProportional(outerloop * edge_threshold);

            for(Vertex vertex:graph.getGraph().vertexSet()){
                double sum_above = 0;
                double sum_below = 0.000001;
                for(Vertex neighbor:vertex.getNeighbor(graph.getGraph())){
                    double conductivity = graph.getGraph().getEdge(vertex,neighbor).getConductivity();
                    sum_below = sum_below + conductivity / adjmatrix[vertex.getName()][neighbor.getName()];
                    sum_above = sum_above + conductivity * pressure[neighbor.getName()] / adjmatrix[vertex.getName()][neighbor.getName()];
                }

                if(vertex.isSource()){
                    if(vertex.getName()==sink.getName())
                        sum_above = sum_above - (sourceSet.size()-1) * I_0;
                    else
                        sum_above = sum_above + I_0;
                }else
                    ;
                graph.getPressure().get(graph.getSource_list().indexOf(sink.getName()))[vertex.getName()] = sum_above/sum_below;
            }
        }
        return prev_edge_sum;
    }

    public void iteration_ksub_changed(int T_c){
        double sum = 0;
        for(int i=1;i<=T_c;i++) {
            for (Vertex sink : sourceSet) {
                sum = iteration_ksub(1, sink,i);
            }
        }

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
        JGraphXAdapter<Vertex,Edge> graphXAdapter = new JGraphXAdapter<Vertex, Edge>(g);

        mxIGraphLayout layout = new mxOrganicLayout(graphXAdapter);
        layout.execute(graphXAdapter.getDefaultParent());

        frame.add(new mxGraphComponent(graphXAdapter));

        frame.pack();
        frame.setLocationByPlatform(true);
        frame.setVisible(true);
    }

    public double cutEdgeProportional(double edge_threshold){
        List<Edge> edges = new ArrayList<>(graph.getGraph().edgeSet());
        Collection<Edge> edgesToRemove = new HashSet<>();
        for(Edge e:edges){
            Vertex source = (Vertex)e.getSource();
            Vertex target = (Vertex)e.getTarget();
            double source_sum,target_sum;
            source_sum = target_sum = 0;
            for(Edge e1:graph.getGraph().incomingEdgesOf(source))
                source_sum += e1.getConductivity();
            for(Edge e1:graph.getGraph().incomingEdgesOf(target))
                target_sum += e1.getConductivity();
            if(e.getConductivity()<10E-4)
                edgesToRemove.add(e);
            if(e.getConductivity()/source_sum<edge_threshold||e.getConductivity()/target_sum<edge_threshold)    //tweak here
                edgesToRemove.add(e);

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

        ConnectivityInspector<Vertex,Edge> inspector = new ConnectivityInspector<>(this.graph.getGraph());
        if(!inspector.isConnected()){
            List<Set<Vertex>> subgraph = inspector.connectedSets();
            for(Set<Vertex> graph:subgraph){
                boolean flag = false;
                for(Vertex v:graph){
                    if(v.isSource()){
                        flag = true;
                        break;
                    }
                }
                if(!flag){
                    this.graph.getGraph().removeAllVertices(graph);
                }
            }
        }

        double sum = 0;
        for(Edge e:graph.getGraph().edgeSet()){
            sum += e.getWeight();
        }

        return sum;
    }
}
