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
import org.jgrapht.*;
import org.jgrapht.Graph;
//import com.mxgraph.layout.*;
//import com.mxgraph.swing.*;

import javax.swing.*;
import java.util.List;


public class Steiner_Global {
    public SteinerGraph getGraph() {
        return graph;
    }

    private double prev_edge_sum;
    private SteinerGraph graph;
    public Steiner_Global(SteinerGraph graph){
        this.graph = graph;
    }

    public Steiner_Global(SteinerGraph graph,double rho,double edge_threshold){
        this.graph = graph;
        this.rho = rho;
        this.edge_threshold = edge_threshold;
    }

    private double I_0 = 1;   //todo
    private double delta = 0.0012;
    private double rho = 0.06;
    private double edge_threshold = 0.000000004;

    private Set<Vertex> sourceSet;

    public void initial(){

        Set<Vertex> vertexSet = graph.getGraph().vertexSet();
        sourceSet = new HashSet<>();
        Iterator<Vertex> iterator = vertexSet.iterator();
        while(iterator.hasNext()){  //initial the source set
            Vertex v = iterator.next();
            if(v.isSource()) {
                sourceSet.add(v);
            }
        }
        Iterator<Vertex> iterator_source = sourceSet.iterator();
        Iterator<Vertex> iterator_neighbor;
        Vertex curr_vertex,curr_source;

/***-------------------------init the matrix----------------------***/
        double[][] adjmatrix = this.graph.getAdjmatrix();
        while(iterator_source.hasNext()){
            iterator = vertexSet.iterator();
            curr_source = iterator_source.next();
            graph.getSource_list().add(curr_source.getName());
            double[][] coefficients = new double[graph.getGraph().vertexSet().size()][graph.getGraph().vertexSet().size()];
            double[] constant = new double[graph.getGraph().vertexSet().size()];

            while(iterator.hasNext()){
                curr_vertex = iterator.next();
                if(curr_vertex.isSource()){     //is source
                    if(curr_vertex.getName()==curr_source.getName()){   //is Vk & source
                        iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                        while(iterator_neighbor.hasNext()){  //for each neighbor
                            Vertex curr_neighbor = iterator_neighbor.next();
                            coefficients[curr_vertex.getName()][curr_neighbor.getName()] = 1;
                            constant[curr_vertex.getName()] = (graph.getSource_num()-1) * I_0 * adjmatrix[curr_vertex.getName()][curr_neighbor.getName()];  //or upside down?
                        }
                    }else{      //is source but not Vk
                        iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                        constant[curr_vertex.getName()] = I_0 ;
                        coefficients[curr_vertex.getName()][curr_vertex.getName()] = curr_vertex.getNeighbor(graph.getGraph()).size();
                        while(iterator_neighbor.hasNext()){
                            coefficients[curr_vertex.getName()][iterator_neighbor.next().getName()] = -1;
                        }
                    }
                }else {      //not source
                    constant[curr_vertex.getName()] = 0;
                    iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                    coefficients[curr_vertex.getName()][curr_vertex.getName()] = curr_vertex.getNeighbor(graph.getGraph()).size();
                    while (iterator_neighbor.hasNext()) {
                        int i = curr_vertex.getName();
                        int j = iterator_neighbor.next().getName();
                        coefficients[i][j] = -1;
                    }
                }


            }
            RealMatrix matrix = new Array2DRowRealMatrix(coefficients,false);
            DecompositionSolver solver = new QRDecomposition(matrix).getSolver();
            RealVector vector = new ArrayRealVector(constant);
            graph.getPressure().add(solver.solve(vector).toArray());
            System.out.println("Done calculating pressure with vertex " + curr_source.getName()+" as the source");
        }
/***---------------------------init the matrix---------------------***/

    }

    public void initial_changed(){
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
                coefficients[curr_vertex.getName()][curr_vertex.getName()] = P_i_coefficient;
            }

            RealMatrix matrix = new Array2DRowRealMatrix(coefficients,false);
            DecompositionSolver solver = new QRDecomposition(matrix).getSolver();
            RealVector vector = new ArrayRealVector(constant);
            graph.getPressure().add(solver.solve(vector).toArray());
            System.out.println("Done calculating pressure with " + sink.getName() + " as sink.");
        }
    }

    public double iteration(int T_c){

        for(int i=0;i<T_c;i++){     //for each loop
            Set<Vertex> vertexSet = graph.getGraph().vertexSet();
            Iterator<Vertex> iterator_vertex = vertexSet.iterator();

            while(iterator_vertex.hasNext()){   //for each vertex V_i
                Vertex curr_vertex = iterator_vertex.next();
                Iterator<Vertex> iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                double sum;

                while(iterator_neighbor.hasNext()){     //for each neighbor of V_i, AKA V_j, update conductivity according to Eq6
                    sum = 0;
                    Vertex curr_neighbor = iterator_neighbor.next();
                    Edge curr_edge = graph.getGraph().getEdge(curr_vertex,curr_neighbor);
                    double curr_conductivity = curr_edge.getConductivity();
                    for(int j=0;j<graph.getPressure().size();j++){
                       sum = sum + graph.getPressure().get(j)[curr_vertex.getName()] - graph.getPressure().get(j)[curr_neighbor.getName()];
                    }
                    sum = Math.abs(sum);
                    double K_ij = 1 + delta * sum - rho * graph.getAdjmatrix()[curr_vertex.getName()][curr_neighbor.getName()];
                    curr_edge.setConductivity(K_ij * curr_conductivity);
                }

                Iterator<Vertex> iterator_source = this.sourceSet.iterator();
                while(iterator_source.hasNext()){
                    Vertex curr_source = iterator_source.next();
                    if(curr_vertex.isSource()){     //is source
                        if(curr_source.getName() == curr_vertex.getName()){     //is source & Vk
                            graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()))[curr_vertex.getName()] = 0;
                        }else{      //is source but not Vk
                            double sum_above = I_0;
                            double sum_below = 0;
                            iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                            while(iterator_neighbor.hasNext()){
                                Vertex curr_neighbor = iterator_neighbor.next();
                                double[] pressure = graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()));
                                Edge curr_edge = graph.getGraph().getEdge(curr_vertex,curr_neighbor);
                                sum_above = sum_above + curr_edge.getConductivity() * (pressure[curr_vertex.getName()] + pressure[curr_neighbor.getName()]);
                                sum_below = sum_below + 2 * curr_edge.getConductivity();
                            }
                            graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()))[curr_vertex.getName()] = sum_above / sum_below;
                        }
                    }else{      //not source
                        double sum_above = 0;
                        double sum_below = 0;
                        iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                        while(iterator_neighbor.hasNext()){
                            Vertex curr_neighbor = iterator_neighbor.next();
                            double[] pressure = graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()));
                            Edge curr_edge = graph.getGraph().getEdge(curr_vertex,curr_neighbor);
                            sum_above = sum_above + curr_edge.getConductivity() * (pressure[curr_vertex.getName()] + pressure[curr_neighbor.getName()]);
                            sum_below = sum_below + 2 * curr_edge.getConductivity();
                        }
                        graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()))[curr_vertex.getName()] = sum_above / sum_below;
                    }
                }

            }

            ConnectivityInspector<Vertex,Edge> inspector = new ConnectivityInspector<>(this.graph.getGraph());
            if(!inspector.isConnected()){
                List<Set<Vertex>> subgraph = inspector.connectedSets();
                int count = 0;
                for(Set<Vertex> graph:subgraph){
                    for(Vertex v:graph){
                        if(v.isSource()) {
                            count++;
                            break;
                        }
                    }
                }
                if(count>1){
                    System.out.println("Max loop time: " + (i-1));
                    return prev_edge_sum;
                }
            }

            prev_edge_sum = graph.getEdgeWeightSum();

            this.cutEdgeProportional(edge_threshold*i);

        }
        return prev_edge_sum;       //potential bug here
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

    public double cutEdge(double edge_threshold){
        Set<Edge> edgeSet = graph.getGraph().edgeSet();

        List<Edge> edges = new ArrayList<>(edgeSet);

        for(int i=0;i<edges.size();i++){
            if(edges.get(i).getConductivity()<edge_threshold)
                graph.getGraph().removeEdge(edges.get(i));
        }


        Iterator<Vertex> iterator_vertex = graph.getGraph().vertexSet().iterator();
        Collection<Vertex> vertexToRemove = new HashSet<>();
        while(iterator_vertex.hasNext()){
            Vertex curr_vertex = iterator_vertex.next();
            if(curr_vertex.getNeighbor(graph.getGraph()).size() == 0){
                vertexToRemove.add(curr_vertex);
            }
        }
        graph.getGraph().removeAllVertices(vertexToRemove);

        double sum = 0;
        for(Edge e:graph.getGraph().edgeSet()){
            sum += e.getWeight();
        }

        System.out.println("Sum of edges: " + sum);
        return sum;
    }

    public double cutEdgeProportional(double edge_threshold){
        List<Edge> edges = new ArrayList<>(graph.getGraph().edgeSet());
        Collection<Edge> edgesToRemove = new HashSet<>();
        for(Edge e:edges){
            Vertex source = (Vertex)e.getSource();
            Vertex target = (Vertex)e.getTarget();
            double source_sum,target_sum,P_source,P_target;
            P_source = 1/(graph.getGraph().incomingEdgesOf(source).size()*(1.2-edge_threshold));
            P_target = 1/(graph.getGraph().incomingEdgesOf(target).size()*(1.2-edge_threshold));
            source_sum = target_sum = 0;
            for(Edge e1:graph.getGraph().incomingEdgesOf(source))
                source_sum += e1.getConductivity();
            for(Edge e1:graph.getGraph().incomingEdgesOf(target))
                target_sum += e1.getConductivity();
            if(e.getConductivity()/source_sum<P_source&&e.getConductivity()/target_sum<P_target)
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

        double sum = 0;
        for(Edge e:graph.getGraph().edgeSet()){
            sum += e.getWeight();
        }

        System.out.println("Sum of edges: " + sum);
        return sum;
    }
}
