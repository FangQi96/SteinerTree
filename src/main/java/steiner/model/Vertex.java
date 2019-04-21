package steiner.model;

import org.jgrapht.Graph;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import static org.jgrapht.Graphs.predecessorListOf;

public class Vertex {

    public void setServer(DistributedServer server) {
        this.server = server;
    }

    public Thread getServerThread() {
        return serverThread;
    }

    public void setServerThread(Thread serverThread) {
        this.serverThread = serverThread;
    }

    private Thread serverThread;
    private DistributedServer server;
    private HashMap<Integer,Vertex> neighbors = new HashMap<>();

    public int getName() {
        return name;
    }

    private pressureLocal pressureLocal;

    public pressureLocal getPressureLocal(){
        return this.pressureLocal;
    }

    public void setPressureLocal(ArrayList<Double> pressure,int turn){
        this.pressureLocal.setPressureLocal(pressure,turn);
    }

    private ConcurrentHashMap<Vertex,pressureLocal> neighborsPressureLocal = new ConcurrentHashMap<>();

    public ConcurrentHashMap<Vertex,pressureLocal> getNeighborsPressureLocal() {
        return neighborsPressureLocal;
    }

    private final int name;

    public boolean isSource() {
        return isSource;
    }

    private boolean isSource;

    public Vertex getNeighborByName(int neighborName){
        return this.neighbors.get(neighborName);
    }

    public List<Vertex> getNeighbor(Graph<Vertex,Edge> graph){
        List neighbors = predecessorListOf(graph,this);
        return neighbors;
    }

    public Set<Vertex> getNeighbors(SteinerGraph steinerGraph){
        Set<Vertex> neighbors = new HashSet<>();
        Set<Vertex> vertexSet = steinerGraph.getGraph().vertexSet();
        this.neighbors.clear();
        for(Vertex vertex:vertexSet){
            if(steinerGraph.getGraph().getEdge(this,vertex)!=null) {
                neighbors.add(vertex);
                this.neighbors.put(vertex.getName(),vertex);
            }
        }
        return neighbors;
    }

    public void pullNeighborPressure(SteinerGraph steinerGraph) throws IOException {
        neighborsPressureLocal.clear();
        for(Vertex neighbor:this.getNeighbors(steinerGraph)){
            this.server.ask(neighbor.server.getPORT());
        }
    }

    public Vertex(int name, boolean isSource) throws IOException {
        this.name = name;
        this.isSource = isSource;
        pressureLocal = new pressureLocal(0);
    }

    public Vertex(int name){
        this.name = name;
        this.isSource = false;
        pressureLocal = new pressureLocal(0);
    }

    public String toString(){   //when using for visualization, the vertex's index begins from 1
        if(this.isSource())
            return "SRC " + (name+1);
        else
            return "" + (name+1);
    }

    public int hashCode(){
        return toString().hashCode();
    }

    public boolean equals(Object o){
        return (o instanceof Vertex) && (toString().equals(o.toString()));
    }
}
