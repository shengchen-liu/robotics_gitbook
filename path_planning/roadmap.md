# Roadmap

Roadmap represents the configuration space using a simple connected graph - similar to how a city can be represented by a metro map.

![](assets/c5-l2-41-img-subway-map-v1.png)

Roadmap methods are typically implemented in two phases:

**Construction**: builds up a graph from a continuous representation of the space. This phase usually takes a significant amount of time and effort, but the resultant graph can be used for multiple queries with minimal modifications.

**Query**: evaluates the graph to find a path from a start location to a goal location. This is done with the help of a search algorithm.

Two roadmap methods will be disucussed:

1. Visibility graph
2. Voronoi Diagram