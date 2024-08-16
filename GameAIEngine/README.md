[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/6b2XrJ3k)
# EAE6610: Assignment3-DecisionMaking

## Instructions
Your task for this assignment is to explore decision trees, behavior trees, and goal-oriented action planning. Working alone and using openFrameworks (http://openframeworks.cc), your task is to implement the algorithms described below and analyze your results in a 3-5 page writeup. As before, you will be integrating your new solutions with your code from Assignments 1 and 2.

You are expected to follow Google’s C++ Style Guide for your code. It is available here: https://google.github.io/styleguide/cppguide.html

This is an individual assignment; you are to work alone. As always, you are expected to abide by the University of Utah **Code of Conduct**, explicitly outlined in the class syllabus, which includes providing appropriate attribution for all external sources of information consulted while working on this assignment.

### Overview
1. [Setup](#Setup)
2. [Assignment Deliverables](#assignment-deliverables)
    1. [Decision Trees](#decision-trees-1-point)
    2. [Behavior Trees](#behavior-trees-3-points)
    3. [Goal Oriented Action Planning](#goal-oriented-action-planning-3-points)
    4. [Writeup](#writeup-5-points)
    5. [Peer Review](#peer-review-3-points)
    6. [Submission Instructions](#submission-instructions)

### Setup
For this assignment, you will be using your movement and pathfinding code from Assignments 1 and 2 to demonstrate higher-level decision making. To start, ensure you have functioning pathfinding algorithms and basic movement algorithms including *Seek* and *PathFollowing*. Additionally, make sure that your environment from the “Putting It All Together” section of Assignment 2 is functioning for pathfinding/pathfollowing.

### Assignment Deliverables

#### Decision Trees (1 Point)
Your first task for this assignment is to incorporate decision trees into your movement and pathfinding system. Your task is to devise an ***abstraction scheme / parameterization*** of your environment that will enable you to build decision trees as we discussed in class. Your decision tree should allow your character to decide on changing between pathfinding and 2 or more movement behaviors, within a game level like the one needed for Assignment 2. For example, you may decide that:
- Whenever your character reaches maximum velocity, its behavior should change to wander.
- Whenever your character wanders near a wall, its behavior should change to pathfind to a particular target location (like the center of a room).
- Whenever your character performs seeks a target for 3 seconds, the target location should randomly change.

Your decision tree should have **3 layers of decisions**. It should primarily be responsible for the decisions made about when and where to update target locations, when to path-find to those targets, when to change movement behaviors, and the like. Use your imagination and more importantly (as always) write about it!

**The challenge in this part of the homework is not the algorithm, but the parameterization itself: the choice of how to represent the state/information of the environment.**

Make sure you include the variables you will need for your decision tree to encapsulate the choices you want it to make. Make sure you devote a significant portion of your writeup on this section to addressing this point. It is crucial! Be sure to also include a diagram (for example, a visual figure) of the decision tree itself in your writeup.

#### Behavior Trees (3 Points)
Your task for this section of the assignment is to implement a behavior tree algorithm; you may follow the implementation we discussed in class or develop some other architecture. In any case, you should describe the algorithm conceptually in your write up. ***You must have at least four types of nodes in your behavior tree*** (e.g. Sequence, Selector, Decorator, Nondeterministic Sequence/Selector, Parallel, or something else you come up with). Your parameterization for the Behavior Tree should match what you developed for your Decision Trees. The intent is that both your Decision Tree and Behavior Tree rely on the same abstraction scheme.

The behavior tree should control **a “monster” character**. Using your behavior tree and movement algorithms from your previous assignment, incorporate a “monster” into your environment. Your monster’s task should be to move around the environment and try to “eat” your character. Also, have your monster do something else interesting (perhaps a little dance or following a scripted path). Use your behavior tree to encode all of this behavior. If your monster is successful at eating your character, have them both return to neutral starting positions and begin again.

Make sure to both (a) describe the behavior in words and (b) include a diagram of the behavior tree in your writeup. If you invent your own type of behavior tree composite task, make sure to clearly explain it as well. Was it hard to author the behavior tree? Was it hard to implement? Did your monster behave as you expected? Write it up!

#### Goal Oriented Action Planning (3 Points) 
For the last part of this assignment, you will implement a goal-oriented action planning algorithm; you may follow the implementation we discussed in class or develop some other representation. In any case, you should describe the algorithm conceptually in your write up.

The planner should control **your character**. You will co-run the planner for your character and the behavior tree for your monster. Your planner should prompt a response from the behavior tree-controlled monster and will hopefully lead to interesting dynamics between them. As with Decision Trees, **the challenge in this part of the homework is not the algorithm, but the *parameterization* itself: the choice of how to represent the state/information of the environment. You must use an abstraction scheme for your planner that is _different_ from the one used for your Decision Tree / Behavior Tree.** That is, your planner must consider different aspects of the world in its processing. Make sure that the parameterization includes *something* that describes your character’s state and something that describes your monster's state. Then, set up at *least* two goals that are achievable given your states and actions.

You can use a binary-valued vector, number-valued vector, or some other structure for your state representation. Examples of attributes include: the room the character is in, the orientation of the character, the number of other characters in the room, distance to the closest obstacle in a particular direction, time since the last collision with a monster, and so on. The parameters that make up your state representation will then be reflected in the structure of your actions: your preconditions, add-effects, and delete-effects will all reference the state.

How do the characters interact? Do the differently-controlled characters look qualitatively different? Please discuss your GOAP abstraction scheme / parameterization in your writeup, and contrast it to your Behavior Tree / Decision Tree one.

#### Writeup (5 Points)
Now that you have implemented and evaluated a number of algorithms, write a 3-5 page paper summarizing your findings. Your paper must be formatted with the ToDIGRA submission template, available here: http://todigra.org/index.php/todigra/about/submissions. Note, 3-5 pages means at least **three full pages of content**. Please name your writeup per the convention:
*[last name]_assignment[number]_report.pdf*

It is ***strongly*** recommended that you do not limit yourself to only answering those questions posed in this assignment. Think creatively about what you have done. What other parameters can you tweak and what effect do they have on the results? The most successful write-ups will contain evidence that you have thought deeply about these algorithms and what they can produce and have gone beyond what is written in the assignment.

Please include all relevant screenshots to support your analysis. The screenshots do not count toward your 3-5 page requirement. Making connections between your writeup and other class materials will be looked upon _very_ favorably as part of class participation.

#### Peer Review (3 Points)
This homework includes a peer-review component where you will anonymously review and pregrade someone outside of your project team **per the Deliverables Rubric described below**. In turn, a student outside of your project team will anonymously review and pre-grade your assignment based on the same rubric. I will ultimately review the peer assessments and override the grade if I perceive there are issues in the peer-based feedback.

> [!TIP]
> Note: If you perceive there is an issue with a peer’s review of your work, please let me know and we can proceed with an appeal / re-grading process. I reserve the right to certify a *pregrade* as the Official Grade™ for the assignment.

There are two reasons for including this peer review component:
- Peer review is the norm within the games industry, and this is one way you can practice evaluating benefits and drawbacks of different ways to implement Game AI techniques (Learning Objective 4)
- There’s no better way to pick up new tricks for doing the same techniques! No one is as smart as all of us.

In your pre-grading, **you must offer justification and feedback/comments for every score you assign your peer**. This is *especially* needed in the edge cases: when you assign a score of 0 for a section, or assign a perfect score for a section. If it’s perfect, focus on what stretch goals might have been cool to see, or offer praise for particularly insightful ways they went about completing the task (and why you think it’s insightful). If it’s 0, focus on what aspects your peer could improve on and encourage them with advice, words of wisdom, lessons learned, and resources you have found to be helpful to you.

#### Submission Instructions
There are two submissions for this assignment: **your deliverables** and **your peer-review**.

### Submitting Your Deliverables
**_By the assignment deadline_** stated on the syllabus, please provide a link to a Github or Gitlab repository that we can access to grade. The repository should contain all of your code (potentially including the openFrameworks library itself), and should also contain your writeup in .pdf format.

Your repository **must** include a README text/Markdown file containing a description of how to run your code, as well as where to find the source code relevant to each of the sections identified above, as well as any special instructions we need to consider to run your code.

#### Submitting Your Peer Review
_**After the assignment deadline**_ stated on the syllabus, you will be randomly assigned to anonymously review someone else in the class via Canvas. You are expected to perform your peer-review on Canvas at your earliest convenience, **but no later than the deadline for the next class deliverable that is due.**

To complete your peer-review, go to the Deliverables Rubric detailed below, and for each item, decide whether to assign full, partial, or no credit to your peer based on their submitted deliverables. You must also apply multipliers where relevant, and justify accordingly. For each credit assignment, please justify your choice and offer comments/feedback for improvement. Note: you will not be peer-reviewing your peer’s peer-review.

## Rubric
Your deliverables will be evaluated according to the rubric described below.

### Decision Trees (1 Point).
- (0.75)	The Decision Tree controls something related to switching within Dynamic Movement behaviors, Pathfinding behaviors, and between them.
- (0.25)	The Decision Tree has three layers of decisions (the Decision Tree has a height of at least 4).
  
### Behavior Trees (3 Points).
- (1.0)	The Behavior Tree has four types of nodes in the tree.
- (1.0)	The Behavior Tree controls a “monster” that tries to eat a character (e.g. Boid).
- (1.0)	The Behavior Tree uses the same abstraction scheme / parameterization used for the Decision Tree. This means: all information used to arrive at a decision for the Decision Tree is used *somewhere* in the Behavior Tree.

### Goal Oriented Action Planning (3 Points).
- (1.0) The planner controls the non-monster character.
- (0.5) The planner uses a *different* abstraction scheme used for the Decision/Behavior Trees.
- (0.25) The parameterization includes some aspect of the Boid’s state.
- (0.25) The parameterization includes some aspect of the “monster”’s state.
- (1.0) The GOAP Planner can accomplish at least two goals given its parameterization.

### Writeup (5 Points)
- (0.5) Describes the Decision Tree behavior in words.
- (0.25) Decision Tree structure is illustrated via a diagram.
- (0.75) The description of the abstraction scheme / parameterization used for both Decision Trees and Behavior Trees includes what variables are needed for the decisions it makes.
- (0.50) Describes the chosen Behavior Tree architecture and implementation.
- (0.50) Describes the Behavior Tree behavior in words.
- (0.25) Behavior Tree structure is illustrated via a diagram.
- (0.50) Describes the chosen GOAP architecture and implementation.
- (0.50) Describes the GOAP behavior in words.
- (0.50) Goal Oriented Action Planning questions posed in the assignment are answered.
- (0.75) The description of the abstraction scheme / parameterization used for GOAP clearly discusses how it is different from the Decision Tree / Behavior Tree one.

**The Walls Are Lava™ (Variable multiplier)**. If the Boid or monster does touch the wall during moving, the _**total score**_ is slightly reduced:
- If the Boid has **little overlap** with walls, your total score will be multiplied by 0.95.
- If the Boid has **moderate overlap** with walls, your total score will be multiplied by 0.85.
- If the Boid has **significant overlap** with walls, your total score will be multiplied by 0.75.

## Multipliers

### Codebase Style
- **No README (×0 multiplier)**. The Github / Gitlab repository does not have a README file.
- **Does Not Compile / Build (×0 multiplier)**. The code submission does not compile per the instructions on the README.

### Writeup Style
- **Format (×0 multiplier)**. The writeup does not adhere to the ToDIGRA submission template, available here: http://todigra.org/index.php/todigra/about/submissions. Note: you are not submitting this for anonymous review. :)
- **Page Count (Variable multiplier)**. Your writeup must hit at least 3 pages of content. If you have less than 3, then the writeup score will be multiplied by (n / 3), where n is the teaching staff’s estimate of how much page content you have.
- **Spelling and Grammar (Variable multiplier)**. Having proper grammar and spelling is important and difficult. If you are having trouble, ask for help from your classmates or the university writing center.
    - If you have 1-3 grammar/spelling mistakes, your score will be multiplied by 0.95.
    - If you have 4-6 grammar/spelling mistakes, your score will be multiplied by 0.85.
    - If you have 7+ grammar/spelling mistakes, your score will be multiplied by 0.75.


