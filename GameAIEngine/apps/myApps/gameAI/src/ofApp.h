#pragma once

#include "ofMain.h"
#include <chrono> // For time measurement

class Vector {
public:
	float x, y;

	Vector(float _x = 0, float _y = 0) : x(_x), y(_y) {}

	Vector operator+(const Vector& other) const {
		return Vector(x + other.x, y + other.y);
	}

	Vector operator-(const Vector& other) const {
		return Vector(x - other.x, y - other.y);
	}

	Vector operator*(float scalar) const {
		return Vector(x * scalar, y * scalar);
	}

	Vector operator/(float scalar) const {
		return Vector(x / scalar, y / scalar);
	}

	Vector& operator *= (float scalar) {
		x *= scalar;
		y *= scalar;
		return *this;
	}

	float magnitude() const {
		return sqrt(x * x + y * y);
	}

	void normalize() {
		float mag = sqrt(x * x + y * y);
		if (mag != 0) {
			x /= mag;
			y /= mag;
		}
	}

	void set(float x_, float y_) {
		x = x_;
		y = y_;
	}

	void limit(float maxMagnitude) {
		float magnitude = sqrt(x * x + y * y);
		if (magnitude > maxMagnitude && magnitude != 0) {
			float scaleFactor = maxMagnitude / magnitude;
			x *= scaleFactor;
			y *= scaleFactor;
		}
	}
};

class Kinematic {
public:
	Vector position;
	float orientation;
	Vector velocity;
	float rotation;

	Kinematic(Vector _position = Vector(), float _orientation = 0, Vector _velocity = Vector(), float _rotation = 0) :
		position(_position), orientation(_orientation), velocity(_velocity), rotation(_rotation) {}
};

class SteeringOutput {
public:
	Vector linear;
	float angular;

	SteeringOutput(Vector _linear = Vector(), float _angular = 0) : linear(_linear), angular(_angular) {}
};

class Boid {
public:
	Kinematic kinematic;

	Boid(Vector _position = Vector(), float _orientation = 0, Vector _velocity = Vector(), float _rotation = 0) :
		kinematic(_position, _orientation, _velocity, _rotation) {}

	void update(SteeringOutput output) {
		// Updating velocity and position
		kinematic.velocity = kinematic.velocity + (output.linear * ofGetLastFrameTime());
		kinematic.rotation += output.angular * ofGetLastFrameTime();
		kinematic.position = kinematic.position + (kinematic.velocity * ofGetLastFrameTime());
		kinematic.orientation += kinematic.rotation * ofGetLastFrameTime();

		if (kinematic.position.x > ofGetWidth()) {
			kinematic.position.x = 0;
		}
		else if (kinematic.position.x < 0) {
			kinematic.position.x = ofGetWidth();
		}

		if (kinematic.position.y > ofGetHeight()) {
			kinematic.position.y = 0;
		}
		else if (kinematic.position.y < 0) {
			kinematic.position.y = ofGetHeight();
		}
	}

	void kinematicUpdate(SteeringOutput output) {
		kinematic.velocity = output.linear;
		kinematic.rotation = output.angular;
		kinematic.position = kinematic.position + (kinematic.velocity * ofGetLastFrameTime());
		kinematic.orientation += kinematic.rotation * ofGetLastFrameTime();
	}

	void draw() {
		ofPushMatrix();
		ofTranslate(kinematic.position.x, kinematic.position.y);
		ofRotateZDeg(kinematic.orientation);

		// Drawing Boid
		// Drawing Circle
		ofSetColor(255, 255, 255);
		ofDrawCircle(0, 0, 15);

		// Drawing Triangle
		ofSetColor(255, 255, 255);
		ofDrawTriangle(0, -15, 0, 15, 25, 0);

		ofPopMatrix();
	}

	void leaderDraw() {
		ofPushMatrix();
		ofTranslate(kinematic.position.x, kinematic.position.y);
		ofRotateZDeg(kinematic.orientation);

		// Drawing Boid
		// Drawing Circle
		ofSetColor(255, 0, 0);
		ofDrawCircle(0, 0, 20);

		// Drawing Triangle
		ofSetColor(255, 0, 0);
		ofDrawTriangle(0, -20, 0, 20, 35, 0);

		ofPopMatrix();
	}
};

class Node {
public:
	int id;
	float x, y; // Coordinates
	int dist; // Distance from the source node
	bool visited; // Visited flag
	vector <pair<int, int>> neighbors; // List of neighboring nodes and their weights
	bool obstacle;
};

class Edge {
public:
	int source, sink; // IDs of connected nodes
	float weight; // Distance or travel time
};

class NodeRecord {
public:
	int id;
	Edge connection;
	float costSoFar;
	float estimatedTotalCost;
	// Implement getters and setters as needed
};

class PathfindingList {
private:
	vector<NodeRecord> records;
public:
	// Implement methods to add, remove, and find elements in the list
	void add(NodeRecord record) {
		records.push_back(record);
	}

	bool contains(int id) {
		for (const NodeRecord& record : records) {
			if (record.id == id) {
				return true;
			}
		}
		return false;
	}

	NodeRecord find(int id) {
		for (const NodeRecord& record : records) {
			if (record.id == id) {
				return record;
			}
		}
		throw logic_error("Node not found in PathfindingList.");
	}

	void remove(int id) {
		records.erase(remove_if(records.begin(), records.end(), [id](const NodeRecord& record) {
			return record.id == id;
			}), records.end());
	}

	NodeRecord smallestElement() {
		float minCost = numeric_limits<float>::max();
		NodeRecord smallest;
		for (const NodeRecord& record : records) {
			if (record.costSoFar < minCost) {
				minCost = record.costSoFar;
				smallest = record;
			}
		}
		return smallest;
	}

	NodeRecord smallestElementAStar() {
		assert(!records.empty() && "PathfindingList is empty");
		NodeRecord smallest = records.front();
		for (const auto& record : records) {
			if (record.estimatedTotalCost < smallest.estimatedTotalCost) {
				smallest = record;
			}
		}
		return smallest;
	}

	bool empty() const {
		return records.empty();
	}

	int size() {
		return records.size();
	}
};

class Heuristic {
public:
	float manhattanEstimate(Node node, Node goal) const {
		float dx = abs(node.x - goal.x);
		float dy = abs(node.y - goal.y);
		return dx + dy;
	}

	float euclidianEstimate(Node node, Node goal) const {
		float dx = goal.x - node.x;
		float dy = goal.y - node.y;

		return sqrt((dx * dx) + (dy * dy));
	}
};

class Graph {
public:
	vector<Node> nodes;
	vector<Edge> edges;

	// Adding a node to the graph
	void addNode(int id, float x, float y, bool obstacle) {
		Node node;
		node.id = id;
		node.x = x;
		node.y = y;
		node.obstacle = obstacle;
		nodes.push_back(node);
	}

	// Adding an edge to the graph
	void addEdge(int source, int sink, float weight) {
		Edge edge;
		edge.source = source;
		edge.sink = sink;
		edge.weight = weight;
		edges.push_back(edge);
	}

	vector<Edge> getConnections(const int nodeId) const {
		vector<Edge> connections;
		for (const Edge& edge : edges) {
			if (edge.source == nodeId) {
				connections.push_back(edge);
			}
		}
		return connections;
	}

	int getToNode(const Edge& edge) const {
		int toNode = edge.sink;
		return toNode;
	}

	Node getNodeFromId(int id) {
		for (const Node& node : nodes) {
			if (node.id == id) {
				return node;
			}
		}
	}

	void clear() {
		nodes.clear();
		edges.clear();
	}
};

class DecisionTreeNode{
public:
	virtual DecisionTreeNode* makeDecision() = 0;
};

class Action : public DecisionTreeNode {
public:
	DecisionTreeNode* makeDecision() override {
		return this;
	}
};

class Decision : public DecisionTreeNode {
protected:
	DecisionTreeNode* trueNode;
	DecisionTreeNode* falseNode;
	const Boid& boid;
	const vector <glm::vec2> obstaclePositions;

public:
	Decision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, const Boid& boid, const vector <glm::vec2>& obstaclePositions)
		: trueNode(trueNode), falseNode(falseNode), boid(boid), obstaclePositions(obstaclePositions) {}
	// Method to be overriden in subclasses for specific tests
	virtual bool testValue() = 0;

	// Method to determine which branch to follow based on the test
	DecisionTreeNode* getBranch() {
		if (testValue()) {
			return trueNode;
		}
		else {
			return falseNode;
		}
	}

	// Recursively make decision based on the test result
	DecisionTreeNode* makeDecision() override {
		DecisionTreeNode* branch = getBranch();
		return branch->makeDecision();
	}
};

// Subclass of Decision for a specific test
class VelocityTestDecision : public Decision {
private:
	float maxVelocity;

public:
	VelocityTestDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, const Boid& boid, const vector <glm::vec2>& obstaclePositions, float maxVelocity)
		: Decision(trueNode, falseNode, boid, obstaclePositions), maxVelocity(maxVelocity) {}

	// Testing if the boid's velocity exceeds the maximum
	bool testValue() override {
		float boidVelocity = boid.kinematic.velocity.magnitude();
		return boidVelocity >= maxVelocity;
	}
};

class CloseToObstacleDecision : public Decision {
private:
	const float obstacleProximityThreshold = 50.0f;
public:
	CloseToObstacleDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, const Boid& boid, const vector <glm::vec2>& obstaclePositions)
		: Decision(trueNode, falseNode, boid, obstaclePositions) {}

	bool testValue() override {
		// Checking if the boid is close to any obstacle
		for (const auto& obstacle : obstaclePositions) {
			glm::vec2 boidPosition2D(boid.kinematic.position.x, boid.kinematic.position.y);
			float distance = glm::distance(boidPosition2D, obstacle);
			if (distance < obstacleProximityThreshold) {
				return true;
			}
		}
		return false;
	}
};

class MouseClickDecision :public Decision {
public:
	MouseClickDecision(DecisionTreeNode* trueNode, DecisionTreeNode* falseNode, const Boid& boid, const vector <glm::vec2>& obstaclePositions)
		: Decision(trueNode, falseNode, boid, obstaclePositions) {}

	bool testValue() override {
		return ofGetMousePressed(OF_MOUSE_BUTTON_LEFT);
	}
};

// Example action class
class ChangeToWanderAction : public Action {
public:
	ChangeToWanderAction() {}

	DecisionTreeNode* makeDecision() override {
		std::cout << "Changing behavior to wander\n";
		return Action::makeDecision();
	}
};

class ChangeToPathfindAction : public Action {
public:
	ChangeToPathfindAction() {}

	DecisionTreeNode* makeDecision() override {
		std::cout << "Changing behavior to pathfind\n";
		return Action::makeDecision();
	}
};

class ChangeToRandomPathfindAction : public Action {
public:
	ChangeToRandomPathfindAction() {}

	DecisionTreeNode* makeDecision() override {
		std::cout << "Changing to random path find\n";
		return Action::makeDecision();
	}
};

class PathfindToCenterAction :public Action {
public:
	PathfindToCenterAction() {}

	DecisionTreeNode* makeDecision() override {
		std::cout << "Going To Center\n";
		return Action::makeDecision();
	}
};

class SeekCenterAction :public Action {
public:
	SeekCenterAction() {}

	DecisionTreeNode* makeDecision() override {
		std::cout << "Seeking Center\n";
		return Action::makeDecision();
	}
};

class BehaviorTreeNode {
public:
	std::string nodeName;
	virtual bool run() = 0;
};

// Sequence node to execute its child nodes until one fails
class Sequence :public BehaviorTreeNode {
private:
	std::vector<BehaviorTreeNode*> children;
public:
	Sequence(std::vector<BehaviorTreeNode*> children) :children(children) {}

	bool run() override {
		for (auto child : children) {
			if (!child->run()){
				return false;
			}
			else {
				nodeName = child->nodeName;
			}
		}
		
		return true;
	}
};

// Selector node executes its child nodes in sequence until one succeeds
class Selector :public BehaviorTreeNode {
private:
	std::vector<BehaviorTreeNode*> children;
public:
	Selector(std::vector<BehaviorTreeNode*> children) :children(children) {}

	bool run() override {
		for (auto child : children) {
			if (child->run()) {
				nodeName = child->nodeName;
				return true;
			}
		}
		return false;
	}
};

// Decorator node modifies the behavior of its child node
class Decorator :public BehaviorTreeNode {
protected:
	BehaviorTreeNode* child;
public:
	Decorator(BehaviorTreeNode* child) : child(child) {}
};

// Example of a concrete decorator node (inverts its child's result)
class Inverter :public Decorator {
public:
	Inverter(BehaviorTreeNode* child) :Decorator(child) {}

	bool run() override {
		return !child->run();
	}
};

// Condition Tasks
class PlayerInSight :public BehaviorTreeNode {
private:
	Boid& boid;
	Boid& monsterBoid;
	float range = 100.0f;
public:
	PlayerInSight(Boid& boid, Boid& monsterBoid) : boid(boid), monsterBoid(monsterBoid) {}

	bool run() override {
		// Calculating distance between monster and player
		float dx = boid.kinematic.position.x - monsterBoid.kinematic.position.x;
		float dy = boid.kinematic.position.y - monsterBoid.kinematic.position.y;
		float distance = std::sqrt(dx * dx + dy * dy);

		// Checking if the player is within range
		if (distance < range) {
			return true;
		}
		else {
			return false;
		}
	}
};

class PlayerInRangeToAttack : public BehaviorTreeNode {
private:
	Boid& boid;
	Boid& monsterBoid;
	float range = 30.0f;
public:
	PlayerInRangeToAttack(Boid& boid, Boid& monsterBoid) : boid(boid), monsterBoid(monsterBoid) {}

	bool run() override {
		// Calculating distance between monster and player
		float dx = boid.kinematic.position.x - monsterBoid.kinematic.position.x;
		float dy = boid.kinematic.position.y - monsterBoid.kinematic.position.y;
		float distance = std::sqrt(dx * dx + dy * dy);

		// Checking if the player is within range
		if (distance < range) {
			return true;
		}
		else {
			return false;
		}
	}
};

// Action Tasks
class MoveTowardsPlayer : public BehaviorTreeNode {
public:
	MoveTowardsPlayer() { }

	bool run() override {
		nodeName = "MoveTowardsPlayer";
		std::cout << "Chasing Player\n";
		return true;
	}
};

class Attack :public BehaviorTreeNode {
public:
	Attack() {}

	bool run() override {
		nodeName = "Attack";
		std::cout << "Attacking Player and Resetting To Initial Positions\n";
		return true;
	}
};

class PathfindToRandomPoint : public BehaviorTreeNode {
public:
	PathfindToRandomPoint() {}

	bool run() override {
		nodeName = "PathfindToRandomPoint";
		std::cout << "Pathfinding to random point\n";
		return true;
	}
};

class MonsterBehaviorTree {
private:
	BehaviorTreeNode* root;
public:
	MonsterBehaviorTree(BehaviorTreeNode* root, Boid& boid) : root(root) {}

	void update() {
		root->run(); // Running the behavior tree starting from the root node
	}
};

struct PlayerState {
	Vector position;
	Vector velocity;
	float distanceToObstacle;
	float distanceToMonster;
};

enum class PlayerAction {
	CHANGE_VELOCITY,
	GO_TO_CENTER,
	EVADE_MONSTER,
	PATH_FIND
};

enum class PlayerGoal {
	AVOID_COLLISION,
	REACH_DESTINATION,
	AVOID_MONSTER
};

class PlayerPlanner {
public:

	std::vector<PlayerAction> plan(const PlayerState& state, const std::vector<PlayerGoal>& goals) {
		std::vector<PlayerAction> actions;

		// Checking if the AVOID_COLLISION goal is present
		if (std::find(goals.begin(), goals.end(), PlayerGoal::AVOID_COLLISION) != goals.end()) {
			if (state.distanceToObstacle < 50.0f) {
				// DecisionTreeNode* goToCenter = new PathfindToCenterAction();
				// goToCenter->makeDecision();
				actions.push_back(PlayerAction::GO_TO_CENTER);
			}
		}

		if (std::find(goals.begin(), goals.end(), PlayerGoal::AVOID_MONSTER) != goals.end()) {
			if (state.distanceToMonster < 150.0f) {
				actions.push_back(PlayerAction::EVADE_MONSTER);
			}
		}

		if (std::find(goals.begin(), goals.end(), PlayerGoal::REACH_DESTINATION) != goals.end()) {
			// DecisionTreeNode* pathfind = new ChangeToPathfindAction();
			// pathfind->makeDecision();
			actions.push_back(PlayerAction::PATH_FIND);
		}

		return actions;
	}
};


class ofApp : public ofBaseApp {

public:
	Boid boid;
	Boid monsterBoid;
	Graph graph;
	Graph tileGraph;
	Heuristic heuristic;
	vector <Vector> breadcrumbs;
	vector <Boid> followerBoids;
	vector <Edge> path;
	vector <glm::vec2> obstaclePositions;
	Kinematic target;
	Kinematic monsterTarget;
	int numberOfFollowers = 6;
	int selectedBehavior = -1;
	float obstacleSize = 50.0;
	float closestObstacleDistance = 1000.0f;
	int tileSize = 30;
	int nextPathNodeIndex = 0;

	void setup();
	void update();
	void draw();

	int getRandomCoordinate(int min, int max);
	vector<Edge> dijkstra(const Graph& graph, int start, int goal);
	vector<Edge> AStar(Graph& graph, int start, int goal, Heuristic& heuristic);
	void generateTileGraph();
	void pathfindingToTarget(int mouseX, int mouseY);
	int quantizeToGraph(int x, int y);
	void moveBoidAlongPath(const vector<Edge>& path);
	void ofApp::moveToNextNodeInPath();

	DecisionTreeNode* velocityTest;
	DecisionTreeNode* Wander;
	DecisionTreeNode* Pathfind;
	DecisionTreeNode* RandomPathfind;
	DecisionTreeNode* isMouseClicked;
	DecisionTreeNode* rootNode;
	DecisionTreeNode* isCloseToObstacle;
	DecisionTreeNode* GoToCenter;
	DecisionTreeNode* SeekCenter;

	BehaviorTreeNode* monsterBehaviorTreeRootNode;
	BehaviorTreeNode* sequenceNode1;
	BehaviorTreeNode* sequenceNode2;
	BehaviorTreeNode* sequenceNode3;
	BehaviorTreeNode* inverterNode;
	BehaviorTreeNode* moveTowardsPlayerNode;
	BehaviorTreeNode* attackNode;
	BehaviorTreeNode* pathfindToRandomPointNode;

	PlayerPlanner playerPlanner;
	PlayerState currentState;
	std::vector<PlayerGoal> goals;

	void reviewKinematicMotion();
	void reviewDynamicSeek();
	void reviewDynamicArrive();
	void reviewDynamicWander();
	void reviewFlocking();

	void drawUI();
	void drawGraph(const Graph& graph);
	void drawEnvironment();

	SteeringOutput calculateSteering(Boid boid);
	SteeringOutput calculateLeaderSteering();
	SteeringOutput KinematicMotion();
	SteeringOutput DynamicSeek(Kinematic target, float maxAcceleration);
	SteeringOutput DynamicEvade(Kinematic target, float maxAcceleration);
	SteeringOutput DynamicArrive1(Kinematic target, Boid boid, float maxAcceleration, float maxSpeed, float targetRadius, float slowRadius, float timeToTargetVelocity);
	SteeringOutput DynamicArrive2(Kinematic target, float maxAcceleration, float maxSpeed, float slowRadius, float timeToTargetVelocity);
	SteeringOutput Wander1(Boid boid, float wanderOffset, float wanderRadius, float wanderRate, float wanderOrientation, float maxAcceleration);
	SteeringOutput Wander2(float wanderRadius, float wanderDistance, float wanderAngle, float maxAcceleration);
	SteeringOutput Separation(Boid boid, float maxAcceleration, vector<Boid> targets, float threshold, float decay);
	SteeringOutput VelocityMatch(Boid boid);
	SteeringOutput Flocking(Boid& follower);
	float newOrientation(float current, Vector velocity);
	SteeringOutput Align(Kinematic target, float maxAcceleration, float maxSpeed, float targetRadius, float slowRadius, float timeToTargetRotation);
	float LookWhereYoureGoing();
	float FaceForArrive(Kinematic target, float maxAcceleration);
	SteeringOutput FaceForWander(Kinematic target, float maxAcceleration);
	float randomBinomial();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

};
