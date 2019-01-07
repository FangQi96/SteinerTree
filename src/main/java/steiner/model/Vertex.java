package steiner.model;

import org.jgrapht.Graph;

import java.awt.geom.Point2D;
import java.util.List;

import static org.jgrapht.Graphs.predecessorListOf;

public class Vertex {
    public int getName() {
        return name;
    }

    private final int name;

    private Point2D coordinate = new Point2D.Double();

    public boolean isSource() {
        return isSource;
    }

    public void setSource(boolean source) {
        isSource = source;
    }

    private boolean isSource;


    public List<Vertex> getNeighbor(Graph graph){
        List neighbors = predecessorListOf(graph,this);
        return neighbors;
    }

    public Vertex(int name, boolean isSource){
        this.name = name;
        this.isSource = isSource;
    }

    public Vertex(int name){
        this.name = name;
        this.isSource = false;
    }

    public String toString(){
        return "vertex " + name;
    }

    public int hashCode(){
        return toString().hashCode();
    }

    public boolean equals(Object o){
        return (o instanceof Vertex) && (toString().equals(o.toString()));
    }
}
