package datastructure.graph;

/**
 * Implementation of the edges of a generic Graph data structure composed of vertexes 
 * of type VertexAL containing <code>data</code> objects and implemented using
 * Adjacent Lists. An edge contains a source vertex, a destination vertex and possibly
 * a weight (the default weight is 0)
 * @param <D> type of the data object
 */	
public class EdgeAL<D> implements Edge<D> {

	/** the source vertex of the edge */
	protected VertexAL<D> source;

	/** the destination vertex of the edge */
	protected VertexAL<D> dest;
	
	/** the weight of the edge */
	protected double weight;

	/**
	 * Constructs an edge for a graph implemented using adjacent lists
	 * @param source the source vertex
   	 * @param dest the destination vertex 
	 */					
	public EdgeAL(VertexAL<D> source, VertexAL<D> dest) {
		this.source = source; this.dest = dest; this.weight = 0;
	}
	
	/**
	 * Constructs an edge for a graph implemented using adjacent lists
	 * @param source the source vertex
   	 * @param dest the destination vertex 
   	 * @param weight weight the edge weight 
	 */		
	public EdgeAL(VertexAL<D> source, VertexAL<D> dest, double weight) {
		this.source = source; this.dest = dest; this.weight = weight;
	}
	
	/**
	 * Returns the source vertex of the edge 
	 * @return the source vertex of the edge 
	 */
	public Vertex<D> getSource() {
		return this.source;
	}

	/**
	 * Returns the destination vertex of the edge 
	 * @return the destination vertex of the edge 
	 */
	public Vertex<D> getDest() {
		return this.dest;
	}
	
	/**
	 * Returns the weight of the edge 
	 * @return the weight of the edge 
	 */
	public double getWeight() {
		return this.weight;
	}
	
}
