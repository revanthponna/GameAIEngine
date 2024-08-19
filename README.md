# Game AI Engine

## Part 1: Movement AI

* In the first part of this project, I focused on AI movement algorithms such as Dynamic Arrive, Wander, and Flocking behaviors.
* Implementing these functions made me even more curious than before regarding enemy AI behavior in games, and how we can invent new algorithms for specific needs.

### Dynamic Steering Behavior
* When I first researched the Dynamic Arrive function, I immediately wanted to try implementing it in a simpler way. So during this project, I tried writing a second Arrive function without the need for a target radius.
* Although I didn’t notice any significant changes without the target radius variable, the first implementation definitely looked better to me. Perhaps, including the target radius is just a way to make the algorithm more accurate, since we force acceleration to be zero inside that region.

### Wander Steering Behavior
* Implementing the Wander algorithms was definitely not an easy task, as they relied on the alignment functions, which I struggled with initially.
* However, I still learned how the various toggles like Wander Radius, Wander Offset, and Wander Rate affect the behavior of the boid.

### Flocking & Blending
* I developed the flocking behavior by implementing a blend between 3 functions: Separation, Arrive, and Velocity Match.
* The Separation algorithm makes sure that the followers and leader of the flock do not bump into each other, and always remain at a consistent distance from each other.
* On the other hand, the Arrive and Velocity Match functions ensure that the members of the flock (white) follow the leader (red).

## Part 2: Pathfinding

* The second part of this project was a significant deepdive into how pathfinding works in today’s games. Although I had initially thought of it as just a set of algorithms (A*, Dijkstra, etc.), implementing pathfinding in code helped me discover the various nuances and issues that arise along the way.
* The most challenging part of this assignment was designing the A* algorithm, choosing a right heuristic, and generating the tile graph for the same. There were various points during my work where I had to research and choose correctly how my implementation would look like.

### First Steps
* I decided to go with a simple layout for my graph, with nodes representing key landmarks and edges representing the roads connecting the respective places.

### Dijkstra & A* Algorithms
* In terms of performance, the A* algorithm was definitely more efficient in pathfinding from point to point. The number of nodes discovered were always less in A* compared to Dijkstra.
* Additionally, the runtime of these algorithms also differed based on which graph I was testing on. Generally, A* was much faster in the small graph, as the ‘fill’ of the algorithm was less than that of Dijkstra.

### Heuristics
* My first heuristic was Manhattan Distance, which was primarily underestimating throughout my testing.
* For my second heuristic, I decided to go with Euclidian Distance, as I wanted to compare the performance of A* based on the various ways distance can be calculated between two points. 
* One key property I found was that if there were no obstacles in the path between start and the goal, then euclidian distance was an accurate heuristic. However, in environments where there are many obstacles, it can be less than optimal.

### Abstraction Scheme
* For my abstraction scheme, I decided to implement a tile graph since that seemed the most effective way to incorporate with my graph structure.
* Initially, I designed the tile graph such that nodes were not added in the space where obstacles were present. However, this approach led to various bugs related to vector sizes, since the number of rows and columns would change every time an obstacle was present.
* So, I instead added nodes regardless of the presence of obstacles, but had a bool variable indicating if it is occupying the space of an obstacle. This made it much more easier to quantize the environment and implement A*.


