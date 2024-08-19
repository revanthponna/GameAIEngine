# Game AI Engine


## Part 1: Movement AI

* In the first part of this project, I focused on AI movement algorithms such as Dynamic Arrive, Wander, and Flocking behaviors.
* Implementing these functions made me even more curious than before regarding enemy AI behavior in games, and how we can invent new algorithms for specific needs.

### Dynamic Steering Behavior
* When I first researched the Dynamic Arrive function, I immediately wanted to try implementing it in a simpler way. So during this project, I tried writing a second Arrive function without the need for a target radius.
* Although I didn’t notice any significant changes without the target radius variable, the first implementation definitely looked better to me. Perhaps, including the target radius is just a way to make the algorithm more accurate, since we force acceleration to be zero inside that region.

![Dynamic Arrive](https://github.com/user-attachments/assets/956269c4-6f7c-4ade-971e-abb7ea3d3b5a)

### Wander Steering Behavior
* Implementing the Wander algorithms was definitely not an easy task, as they relied on the alignment functions, which I struggled with initially.
* However, I still learned how the various toggles like Wander Radius, Wander Offset, and Wander Rate affect the behavior of the boid.

![Dynamic Wander](https://github.com/user-attachments/assets/3a2131fc-39e0-45df-b84d-f46ae5a1f616)

### Flocking & Blending
* I developed the flocking behavior by implementing a blend between 3 functions: Separation, Arrive, and Velocity Match.
* The Separation algorithm makes sure that the followers and leader of the flock do not bump into each other, and always remain at a consistent distance from each other.
* On the other hand, the Arrive and Velocity Match functions ensure that the members of the flock (white) follow the leader (red).

![Flocking](https://github.com/user-attachments/assets/8e5fda57-beb3-4f4b-8c58-347cfa6be56f)

## Part 2: Pathfinding

* The second part of this project was a significant deepdive into how pathfinding works in today’s games. Although I had initially thought of it as just a set of algorithms (A*, Dijkstra, etc.), implementing pathfinding in code helped me discover the various nuances and issues that arise along the way.
* The most challenging part of this assignment was designing the A* algorithm, choosing a right heuristic, and generating the tile graph for the same. There were various points during my work where I had to research and choose correctly how my implementation would look like.

### First Steps
* I decided to go with a simple layout for my graph, with nodes representing key landmarks and edges representing the roads connecting the respective places.

![Graph](https://github.com/user-attachments/assets/e9b3af6c-951f-4355-900f-8e43a54a5fc1)

### Dijkstra & A* Algorithms
* In terms of performance, the A* algorithm was definitely more efficient in pathfinding from point to point. The number of nodes discovered were always less in A* compared to Dijkstra.
* Additionally, the runtime of these algorithms also differed based on which graph I was testing on. Generally, A* was much faster in the small graph, as the ‘fill’ of the algorithm was less than that of Dijkstra.

![Dijkstra and AStar](https://github.com/user-attachments/assets/b6174a44-fa0a-428f-a720-32cdee1dbaac)

### Heuristics
* My first heuristic was Manhattan Distance, which was primarily underestimating throughout my testing.
* For my second heuristic, I decided to go with Euclidian Distance, as I wanted to compare the performance of A* based on the various ways distance can be calculated between two points. 
* One key property I found was that if there were no obstacles in the path between start and the goal, then euclidian distance was an accurate heuristic. However, in environments where there are many obstacles, it can be less than optimal.

![Euclidian Distance](https://github.com/user-attachments/assets/7046f5ae-6234-410b-8743-e6ec309eab3a)
![Manhattan Distance](https://github.com/user-attachments/assets/029d4491-2915-4286-bbd9-bf7688efa968)

### Abstraction Scheme
* For my abstraction scheme, I decided to implement a tile graph since that seemed the most effective way to incorporate with my graph structure.
* Initially, I designed the tile graph such that nodes were not added in the space where obstacles were present. However, this approach led to various bugs related to vector sizes, since the number of rows and columns would change every time an obstacle was present.

![Tile Graph](https://github.com/user-attachments/assets/422d4b0b-99c9-4bd0-9ef0-5ff83e434848)
![Path](https://github.com/user-attachments/assets/10af2aca-6cb8-43ab-9041-c905c99ca4be)

* So, I instead added nodes regardless of the presence of obstacles, but had a bool variable indicating if it is occupying the space of an obstacle. This made it much more easier to quantize the environment and implement A*.

![Final Pathfinding](https://github.com/user-attachments/assets/60fc7b1c-743a-46fa-8694-c9bf2729eddc)

## Part 3: Decision Making

* The final part of the Game AI Engine was to explore and implement decision-making algorithms and combine them with the previous parts.

### Decision Trees

![Decision Tree](https://github.com/user-attachments/assets/cc508255-793a-4810-b742-52c74ddefe5a)

* For the abstraction scheme, the main variables I decided to use for my decision and behavior trees were:
1. The velocity of the character/boid - This is for the initial velocity check that determines which decision node to go to next.
2. The locations of all the obstacles in the scene - Data on the obstacles is important because we want to check for distances between the character and obstacles at all times, to avoid bumping into them.
3. Mouse Click - I am not sure how to frame this as an abstraction scheme/parameterization yet, because it is technically not a state of environment, but it is something that the decision tree considers when outputting actions.

### Behavior Trees

![Behavior Tree](https://github.com/user-attachments/assets/968bbfee-eb15-4beb-97d6-c5ba25881c01)

* Although I did not explicitly define a ‘Tick’ for the behavior tree, I found another way of storing node names and using them to implement the respective decision and action tasks.
* This was a really nice discovery, as I was able to update the names accordingly, which made it much simpler and easier to understand for me as a developer.

### Goal-Oriented Action Planning

* For my player character, I defined goals like avoiding the monster, avoiding collisions, and/or reaching a particular destination in the environment.
* The corresponding actions to these goals were also defined, such as pathfinding and evading the monster.
*  Since one of the goals was to avoid the monster, it was essential to take in the monster’s state (position, velocity, orientation) to determine what actions to take at each point. This was different from the decision tree implementation where I only focused on the state of the player and environment.



