package steiner.algo;

import org.apache.commons.math3.linear.*;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import steiner.model.Edge;
import steiner.model.SteinerGraph;
import steiner.model.Vertex;
import java.awt.*;
import java.util.*;

import org.jgrapht.ext.*;
import org.jgrapht.*;
import org.jgrapht.graph.*;
import com.mxgraph.layout.*;
import com.mxgraph.swing.*;

import javax.swing.*;
import java.awt.*;
import java.util.List;


public class Steiner_Global {
    public SteinerGraph getGraph() {
        return graph;
    }

    private SteinerGraph graph;
    private static final Dimension DEFAULT_SIZE = new Dimension(1000,800);
    public Steiner_Global(SteinerGraph graph){
        this.graph = graph;
//        setRho();
    }
    private double I_0 = 1;     //todo
    private double delta = 0.012;
    private double rho = 0.015;
    private double edge_threshold = 0.0000002;

    private void setRho(){
        Set<Edge> edgeSet = this.graph.getGraph().edgeSet();
        for(Edge e:edgeSet){
            rho += e.getWeight();
        }
        rho = 2 / rho;
    }

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
                            constant[curr_vertex.getName()] = (graph.getSource_num()-1) * I_0;
                        }
                    }else{      //is source but not Vk
                        iterator_neighbor = curr_vertex.getNeighbor(graph.getGraph()).iterator();
                        constant[curr_vertex.getName()] = I_0;
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

    public void iteration(int T_c){

/*        int[] source_list = new int[graph.getSource_list().size()];

        for(int i=0;i<source_list.length;i++){
            source_list[i] = graph.getSource_list().get(i);
        }
*/
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
            this.cutEdge(T_c * edge_threshold);
            System.out.println(graph.getPressure().get(0)[2]);
        }
    }

    public void setDelta() {
        Set<Vertex> vertexSet = graph.getGraph().vertexSet();
        Iterator<Vertex> iterator_vertex = vertexSet.iterator();
        double tmp = 0;
        int count = 0;

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
                if(sum>0.001){
                    tmp = tmp + 1/sum;
                    count++;
                }
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
                            sum_above = sum_above + curr_edge.getConductivity() * (/*pressure[curr_vertex.getName()] +*/ pressure[curr_neighbor.getName()]);
                            sum_below = sum_below + /*2 */ curr_edge.getConductivity();
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
                        sum_above = sum_above + curr_edge.getConductivity() * (/*pressure[curr_vertex.getName()] +*/  pressure[curr_neighbor.getName()]);
                        sum_below = sum_below + /*2 */ curr_edge.getConductivity();
                    }
                    graph.getPressure().get(graph.getSource_list().indexOf(curr_source.getName()))[curr_vertex.getName()] = sum_above / sum_below;
                }
            }
        }
        this.delta = tmp / count;

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

    public void cutEdge(double edge_threshold){
        Set<Edge> edgeSet = graph.getGraph().edgeSet();

/*        double max = 0;
        for(Edge e:edgeSet){
            if(e.getConductivity()>max)
                max = e.getConductivity();
        }
        for(Edge e:edgeSet){
            e.setConductivity(e.getConductivity()/max);
        }
*/
        List<Edge> edges = new ArrayList<>(edgeSet);

        for(int i=0;i<edges.size();i++){
            if(edges.get(i).getConductivity()<edge_threshold)
                graph.getGraph().removeEdge(edges.get(i));
//            if(inspector.isConnected())
//                continue;
//           else
//                graph.getGraph().addEdge((Vertex)edges.get(i).getSource(),(Vertex)edges.get(i).getTarget(),edges.get(i));
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

        System.out.println(sum);
    }
}
