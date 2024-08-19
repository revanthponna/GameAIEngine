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



