package datastructure.graph;

import java.util.ArrayList;

/**
 * Interface for a generic Graph data structure composed of vertexes of type Vertex
 * containing <code>data</code> objects and edges of type Edge
 * @param <D> type of the data object
 */
public interface Graph<D> {

	/**
	 * Returns the number of vertexes of the graph
	 * @return the number of vertexes
	 */
	public int vertexNum();

	/**
	 * Returns the number of edges of the graph
	 * @return the number of edges
	 */
	public int edgeNum();

	/**
	 * Returns a list with the vertexes currently in the graph
	 * @return the list with the vertexes currently in the graph
	 */
	public ArrayList<Vertex<D>> vertexes();

	/**
	 *  Returns a list with the edges currently in the graph
	 *  @return the list with the edges currently in the graph
	 */	
	public ArrayList<Edge<D>> edges();

	/**
	 * Returns the out degree of a vertex
     * @param v vertex
	 * @return the out degree of the vertex <code>v</code>
	 */	
	public int outDegree(Vertex<D> v);

	/**
	 * Returns a list with the edges currently exiting from a given vertex
     * @param v vertex
	 * @return the list of edges currently exiting from the vertex <code>v</code>
	 */		
	public ArrayList<Edge<D>> outEdges(Vertex<D> v);

	/**
	 * Returns the edge connecting two given vertexes
	 * (returns null if the edge does not exist)
     * @param x source vertex
     * @param y destination vertex
	 * @return the edge connecting <code>x</code> and <code>y</code> (return null if 
	 * <code>x</code> and <code>y</code> are not connected)
	 */		
	public Edge<D> areAdjacent(Vertex<D> x, Vertex<D> y);

	/**
	 * Add a vertex to the graph
     * @param data the datum to insert in the new vertex
	 * @return the new vertex
	 */		
	public Vertex<D> addVertex(D data);

	/**
	 * Add an edge to the graph
     * @param source the source of the edge to add
     * @param dest the destination of the edge to add	
	 */	
	public void addEdge(Vertex<D> source, Vertex<D> dest);
	
	/**
	 * Add an edge to the graph
     * @param source the source of the edge to add
     * @param dest the destination of the edge to add	
     * @param weight the weight of the edge to add	
	 */	
	public void addEdge(Vertex<D> source, Vertex<D> dest, double weight);

	/**
	 * Remove a vertex
     * @param v the vertex to remove
	 */		
	public void removeVertex(Vertex<D> v);

	/**
	 * Remove an edge
     * @param e the edge to remove
	 */	
	public void removeEdge(Edge<D> e);
	
}
