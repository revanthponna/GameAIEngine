#include "ofApp.h"
#include <unordered_set>

//--------------------------------------------------------------
void ofApp::setup() {
	// Setting up application
	ofSetWindowTitle("Pathfinding");
	ofSetFrameRate(60);
	ofSetBackgroundColor(255);
	ofSeedRandom();
	int numberOfObstacles = 15;
	monsterBoid.kinematic.position = Vector(50.0f, 70.0f);
	ofColor obstacleColor = ofColor(255, 0, 0);
	obstaclePositions = std::vector<glm::vec2>();

	for (int i = 0; i < numberOfObstacles; ++i) {
		float obstacleX = ofRandom(obstacleSize / 2, ofGetWidth() - obstacleSize / 2);
		float obstacleY = ofRandom(obstacleSize / 2, ofGetHeight() - obstacleSize / 2);

		obstaclePositions.push_back(glm::vec2(obstacleX, obstacleY));
	}

	generateTileGraph();
	//selectedBehavior = 0;
	boid = Boid(Vector(50, ofGetHeight() - 80));

	// Adding nodes representing landmarks in SLC
	graph.addNode(0, 100, 100, false);
	graph.addNode(1, 200, 150, false);
	graph.addNode(2, 220, 200, false);
	graph.addNode(3, 400, 70, false);
	graph.addNode(4, 300, 130, false);
	graph.addNode(5, 550, 300, false);
	graph.addNode(6, 480, 220, false);
	graph.addNode(7, 350, 110, false);
	graph.addNode(8, 150, 20, false);
	graph.addNode(9, 600, 330, false);
	graph.addNode(10, 420, 500, false);
	graph.addNode(11, 380, 380, false);
	graph.addNode(12, 300, 600, false);
	graph.addNode(13, 580, 480, false);
	graph.addNode(14, 400, 700, false);

	// Adding edges representing nodes
	graph.addEdge(0, 1, 10);
	graph.addEdge(1, 2, 5);
	graph.addEdge(2, 3, 2);
	graph.addEdge(1, 3, 15);
	graph.addEdge(0, 2, 18);
	graph.addEdge(2, 5, 5);
	graph.addEdge(3, 4, 5);
	graph.addEdge(4, 6, 5);
	graph.addEdge(5, 8, 5);
	graph.addEdge(6, 10, 5);
	graph.addEdge(3, 7, 5);
	graph.addEdge(6, 8, 5);
	graph.addEdge(8, 10, 5);
	graph.addEdge(7, 9, 5);
	graph.addEdge(9, 11, 5);
	graph.addEdge(10, 3, 5);
	graph.addEdge(11, 6, 5);
	graph.addEdge(11, 7, 5);
	graph.addEdge(12, 9, 5);
	graph.addEdge(13, 11, 5);
	graph.addEdge(14, 4, 5);

	Wander = new ChangeToWanderAction();
	Pathfind = new ChangeToPathfindAction();
	RandomPathfind = new ChangeToRandomPathfindAction();
	GoToCenter = new PathfindToCenterAction();
	SeekCenter = new SeekCenterAction();
	isMouseClicked = new MouseClickDecision(Pathfind, SeekCenter, boid, obstaclePositions);
	isCloseToObstacle = new CloseToObstacleDecision(GoToCenter, Wander, boid, obstaclePositions);
	rootNode = new VelocityTestDecision(isMouseClicked, isCloseToObstacle, boid, obstaclePositions, 5.0f);
	// velocityTest = new VelocityTestDecision(action1, action2, boid, 100.0f);

	moveTowardsPlayerNode = new MoveTowardsPlayer();
	attackNode = new Attack();
	pathfindToRandomPointNode = new PathfindToRandomPoint();
	sequenceNode1 = new Sequence({ new PlayerInRangeToAttack(boid, monsterBoid), attackNode });
	sequenceNode2 = new Sequence({ new PlayerInSight(boid, monsterBoid), moveTowardsPlayerNode });
	inverterNode = new Inverter(new PlayerInSight(boid, monsterBoid));
	sequenceNode3 = new Sequence({ inverterNode, pathfindToRandomPointNode });
	monsterBehaviorTreeRootNode = new Selector({ sequenceNode1, sequenceNode2, sequenceNode3 });
	

	/*vector<Edge> AStarpath = AStar(graph, 0, 6, heuristic);
	cout << "AStar Path using Manhattan Distance Heuristic: ";
	for (const Edge& edge : AStarpath) {
		cout << edge.source << " -> ";
	}
	cout << "6" << endl;
	cout << "AStar Path using Euclidian Distance Heuristic: ";
	for (const Edge& edge : AStarpath) {
		cout << edge.source << " -> ";
	}
	cout << "6" << endl;*/
}

SteeringOutput ofApp::calculateSteering(Boid boid) {
	// Calculating steering forces here

	switch (selectedBehavior) {
	case 0:
		return KinematicMotion();
		break;
	case 1:
		return DynamicSeek(target, 30.0f);
		break;
	case 2:
		return DynamicArrive1(target, boid, 100.0f, 138.0f, 5.0f, 100.0f, 0.1f);
		break;
	case 3:
		return Wander1(boid, 1.0f, 0.3f, 2.0f, 1.0f, 2.0f);
		break;
	case 4:
		return Flocking(boid);
		break;
	default:
		break;
	}
}

SteeringOutput ofApp::calculateLeaderSteering() {
	return DynamicArrive1(target, monsterBoid, 100.0f, 138.0f, 5.0f, 100.0f, 0.1f);
}
//--------------------------------------------------------------
void ofApp::update() {
	if (selectedBehavior != -1) {
		switch (selectedBehavior) {
		case 0:
			reviewKinematicMotion();
			break;
		case 1:
			reviewDynamicSeek();
			break;
		case 2:
			reviewDynamicArrive();
			break;
		case 3:
			reviewDynamicWander();
			break;
		case 4:
			reviewFlocking();
			break;
		default:
			break;
		}
	}
	else {

		// DecisionTreeNode* nextNode = rootNode->makeDecision();
		monsterBehaviorTreeRootNode->run();

		currentState.position = boid.kinematic.position;
		currentState.velocity = boid.kinematic.velocity;

		for (const auto& obstacle : obstaclePositions) {
			glm::vec2 boidPosition2D(boid.kinematic.position.x, boid.kinematic.position.y);
			float distance = glm::distance(boidPosition2D, obstacle);
			closestObstacleDistance = min(distance, closestObstacleDistance);
		}
		currentState.distanceToObstacle = closestObstacleDistance;

		float dx = boid.kinematic.position.x - monsterBoid.kinematic.position.x;
		float dy = boid.kinematic.position.y - monsterBoid.kinematic.position.y;
		float distance = std::sqrt(dx * dx + dy * dy);
		currentState.distanceToMonster = distance;

		std::vector<PlayerGoal> goals = { PlayerGoal::AVOID_COLLISION, PlayerGoal::REACH_DESTINATION, PlayerGoal::AVOID_MONSTER };
		std::vector<PlayerAction> actions = playerPlanner.plan(currentState, goals);

		// std::cout << "Planned Actions: " << std::endl;
		for (PlayerAction action : actions) {
			if (static_cast<int>(action) == 1) {
				pathfindingToTarget(400, 400);

				SteeringOutput arriveOutput = DynamicArrive1(target, boid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
				SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
				boid.update(arriveOutput);

				float distanceToTarget = sqrt(((boid.kinematic.position.y - target.position.y) * (boid.kinematic.position.y - target.position.y)) + ((boid.kinematic.position.x - target.position.x) * (boid.kinematic.position.x - target.position.x)));
				if (distanceToTarget <= 40.0f) {
					moveToNextNodeInPath();
				}
			}

			if (static_cast<int>(action) == 2) {
				SteeringOutput evadeOutput = DynamicEvade(monsterBoid.kinematic, 25.0f);
			}

			if (static_cast<int>(action) == 3) {
				SteeringOutput arriveOutput = DynamicArrive1(target, boid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
				SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
				boid.update(arriveOutput);

				float distanceToTarget = sqrt(((boid.kinematic.position.y - target.position.y) * (boid.kinematic.position.y - target.position.y)) + ((boid.kinematic.position.x - target.position.x) * (boid.kinematic.position.x - target.position.x)));
				if (distanceToTarget <= 40.0f) {
					moveToNextNodeInPath();
				}
			}

			// std::cout << static_cast<int>(action) << std::endl;
		}

		// std::cout << monsterBehaviorTreeRootNode->nodeName << endl;
		if (monsterBehaviorTreeRootNode->nodeName == "MoveTowardsPlayer") {
			SteeringOutput arriveOutput = DynamicArrive1(boid.kinematic.position, monsterBoid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
			monsterBoid.update(arriveOutput);
		}

		else if (monsterBehaviorTreeRootNode->nodeName == "Attack") {
			boid.kinematic.position.set(50, ofGetHeight() - 80);
			monsterBoid.kinematic.position.set(50.0f, 70.0f);
		}

		else if (monsterBehaviorTreeRootNode->nodeName == "PathfindToRandomPoint") {
			// std::cout << "Found" << endl;
			// if (monsterBoid.kinematic.velocity.magnitude() < 50.0f) {
				// std::cout << "Choosing" << endl;
				// int randomX = getRandomCoordinate(800, 800);
				// int randomY = getRandomCoordinate(800, 800);
			 	monsterTarget.position.set(400, 400);
				pathfindingToTarget(400, 400);
			// }
			// else {
				// monsterTarget.position.set(200, 300);
				// pathfindingToTarget(200, 300);
			// }

			SteeringOutput arriveOutput = DynamicArrive1(monsterTarget, monsterBoid, 60.0f, 100.0f, 2.0f, 50.0f, 0.1f);
			SteeringOutput seekOutput = DynamicSeek(monsterTarget, 30.0f);
			monsterBoid.update(arriveOutput);

			float distanceToTarget = sqrt(((monsterBoid.kinematic.position.y - monsterTarget.position.y) * (monsterBoid.kinematic.position.y - monsterTarget.position.y)) + ((monsterBoid.kinematic.position.x - monsterTarget.position.x) * (monsterBoid.kinematic.position.x - monsterTarget.position.x)));
			if (distanceToTarget <= 40.0f) {
				moveToNextNodeInPath();
			}
		}

		/*if (dynamic_cast<ChangeToPathfindAction*>(nextNode)) {

			SteeringOutput arriveOutput = DynamicArrive1(target, boid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
			SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
			boid.update(arriveOutput);

			float distanceToTarget = sqrt(((boid.kinematic.position.y - target.position.y) * (boid.kinematic.position.y - target.position.y)) + ((boid.kinematic.position.x - target.position.x) * (boid.kinematic.position.x - target.position.x)));
			if (distanceToTarget <= 40.0f) {
				moveToNextNodeInPath();
			}
			breadcrumbs.push_back(boid.kinematic.position);
		}

		else if (dynamic_cast<PathfindToCenterAction*>(nextNode)) {
			pathfindingToTarget(400, 400);

			SteeringOutput arriveOutput = DynamicArrive1(target, boid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
			SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
			boid.update(arriveOutput);

			float distanceToTarget = sqrt(((boid.kinematic.position.y - target.position.y) * (boid.kinematic.position.y - target.position.y)) + ((boid.kinematic.position.x - target.position.x) * (boid.kinematic.position.x - target.position.x)));
			if (distanceToTarget <= 40.0f) {
				moveToNextNodeInPath();
			}
			breadcrumbs.push_back(boid.kinematic.position);
		}

		else if (dynamic_cast<ChangeToWanderAction*>(nextNode)) {
			SteeringOutput wanderOutput = Wander1(boid, 1.0f, 0.3f, 2.0f, 1.0f, 2.0f);
			boid.update(wanderOutput);
			breadcrumbs.push_back(boid.kinematic.position);
		}

		else if (dynamic_cast<SeekCenterAction*>(nextNode)) {
			target.position.set(400, 400);
			SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
			boid.update(seekOutput);
			breadcrumbs.push_back(boid.kinematic.position);
		}*/

		/*SteeringOutput seekOutput = DynamicSeek(target, 30.0f);
		SteeringOutput arriveOutput = DynamicArrive1(target, boid, 100.0f, 138.0f, 2.0f, 50.0f, 0.1f);
		boid.update(arriveOutput);

		float distanceToTarget = sqrt(((boid.kinematic.position.y - target.position.y) * (boid.kinematic.position.y - target.position.y)) + ((boid.kinematic.position.x - target.position.x) * (boid.kinematic.position.x - target.position.x)));
		if (distanceToTarget <= 40.0f) {
			moveToNextNodeInPath();
		}
		breadcrumbs.push_back(boid.kinematic.position);*/
	}
}

int ofApp::getRandomCoordinate(int min, int max) {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(min, max);

	return distribution(gen);
}

void ofApp::generateTileGraph() {
	tileGraph.clear();
	int numRows = ofGetHeight() / tileSize;
	int numCols = ofGetWidth() / tileSize;
	int id = 0;

	for (int i = 0; i < numRows; ++i) {
		for (int j = 0; j < numCols; ++j) {
			// Calculating the center of the current tile
			float centerX = (j + 0.5) * tileSize;
			float centerY = (i + 0.5) * tileSize;
			id++;
			// Checking if the current tile overlaps with any obstacle
			bool overlapsObstacle = false;
			for (const auto& obstacle : obstaclePositions) {
				if (ofDist(centerX, centerY, obstacle.x, obstacle.y) < obstacleSize) {
					overlapsObstacle = true;
					tileGraph.addNode(id, centerX, centerY, true);
					break;
				}
			}

			// If the tile does not overlap with any obstacle, add a node to the graph
			if (!overlapsObstacle) {

				tileGraph.addNode(id, centerX, centerY, false);
			}
		}
	}

	int numNodes = tileGraph.nodes.size();
	int edgeWeight = 5;

	for (int i = 0; i < numNodes; i++) {
		// Getting the current node
		Node& currentNode = tileGraph.nodes[i];
		// std::cout << currentNode.id << std::endl;

		// Calculating the row and column of the current node in the grid
		int row = i / numCols;
		int col = i % numCols;

		auto isValidNode = [&](int index) {
			// cout << tileGraph.nodes[index].obstacle << endl;
			return index >= 0 && index < tileGraph.nodes.size() && !tileGraph.nodes[index].obstacle;
			};

		auto addEdgeIfValid = [&](int sourceIndex, int targetIndex) {
			if (isValidNode(targetIndex)) {
				tileGraph.addEdge(sourceIndex, targetIndex, edgeWeight);
				// cout << "Valid" << endl;
			}
			/*else {
				cout << "Not Valid" << endl;
			}*/
			};

		if (row > 0) {
			int aboveIndex = (row - 1) * numCols + (col + 1);
			addEdgeIfValid(i + 1, aboveIndex);
			// tileGraph.addEdge(i + 1, aboveIndex, edgeWeight);
		}

		if (row < numRows - 1) {
			int belowIndex = (row + 1) * numCols + (col + 1);
			addEdgeIfValid(i + 1, belowIndex);
			// tileGraph.addEdge(i + 1, belowIndex, edgeWeight);
		}

		if (col > 0) {
			int leftIndex = row * numCols + col;
			addEdgeIfValid(i + 1, leftIndex);
			// tileGraph.addEdge(i + 1, leftIndex, edgeWeight);
		}

		if (col < numCols - 1) {
			int rightIndex = row * numCols + (col + 2);
			addEdgeIfValid(i + 1, rightIndex);
			// tileGraph.addEdge(i + 1, rightIndex, edgeWeight);
		}
	}
}

vector<Edge> ofApp::dijkstra(const Graph& graph, int start, int goal) {
	vector<NodeRecord> nodeRecords(graph.nodes.size());
	PathfindingList open, closed;

	NodeRecord startRecord;
	startRecord.id = start;
	startRecord.costSoFar = 0;
	open.add(startRecord);

	while (!open.empty()) {
		// Finding the smallest element in the open list
		NodeRecord current = open.smallestElement();
		open.remove(current.id);
		closed.add(current);

		// If it is the goal node, then terminate
		if (current.id == goal) {
			// Compiling the list of edges in the path
			vector<Edge> path;
			while (current.id != start) {
				path.push_back(current.connection);
				// Update current to the previous node using edge's fromNode
				current = nodeRecords[current.connection.source];
			}
			reverse(path.begin(), path.end());
			// cout << "Number of Nodes visited by Dijkstra: " << closed.size() << endl;
			return path;
		}

		// Otherwise, get its outgoing edges
		vector<Edge> connections = graph.getConnections(current.id);

		// Looping through each edge in turn
		for (const Edge& connection : connections) {
			int endNodeId = connection.sink;

			// Skip if the node is closed
			if (closed.contains(endNodeId))
				continue;

			// or if it is open and we've found a worse route
			float endNodeCost = current.costSoFar + connection.weight;
			NodeRecord& endNodeRecord = nodeRecords[endNodeId];
			if (!open.contains(endNodeId) || endNodeCost < endNodeRecord.costSoFar) {
				endNodeRecord.id = endNodeId;
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.connection = connection;
				open.add(endNodeRecord);
			}
		}
	}
	// We run out of nodes without finding the goal, so there is no solution
	return {};
}

vector<Edge> ofApp::AStar(Graph& graph, int start, int goal, Heuristic& heuristic) {

	// Initializing the record for the start node
	NodeRecord startRecord;
	startRecord.id = start;
	startRecord.connection = Edge();
	startRecord.costSoFar = 0;
	startRecord.estimatedTotalCost = heuristic.euclidianEstimate(graph.getNodeFromId(start), graph.getNodeFromId(goal));

	// Initializing the open and closed lists
	PathfindingList open;
	open.add(startRecord);
	PathfindingList closed;
	//NodeRecord lastCurrent;

	// Iterating through processing each node
	while (!open.empty()) {
		// Finding the smallest element in the open list (using the estimatedTotalCost)
		NodeRecord current = open.smallestElementAStar();

		// If it is the goal node, then terminate
		if (current.id == goal) {
			vector<Edge> path;
			while (current.id != start) {
				path.push_back(current.connection);
				current = closed.find(current.connection.source);
			}
			reverse(path.begin(), path.end());
			// cout << "Number of nodes visited by A* : " << closed.size() << endl;
			return path;
		}

		// Otherwise, get its outgoing connections
		vector<Edge> connections = graph.getConnections(current.id);

		// Looping through each connection in turn
		for (const auto& connection : connections) {
			// Getting the cost estimate for the end node
			int endNodeId = graph.getToNode(connection);
			float endNodeCost = current.costSoFar + connection.weight;

			NodeRecord endNodeRecord;
			// If the node is closed or if it is open and we've not found a better route, skip.
			if (closed.contains(endNodeId)) {
				endNodeRecord = closed.find(endNodeId);
				if (endNodeRecord.costSoFar <= endNodeCost) {
					continue;
				}
				closed.remove(endNodeRecord.id);
			}

			if (open.contains(endNodeId)) {
				endNodeRecord = open.find(endNodeId);
				if (endNodeRecord.costSoFar <= endNodeCost) {
					continue;
				}
			}

			// Otherwise, we know we got an unvisited node, so make a record for it
			else {
				endNodeRecord.id = endNodeId;
				endNodeRecord.connection = connection;
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.estimatedTotalCost = endNodeCost + heuristic.euclidianEstimate(graph.getNodeFromId(endNodeId), graph.getNodeFromId(goal));
			}
			// And add it to the open list
			if (!open.contains(endNodeId)) {
				open.add(endNodeRecord);
			}
		}

		// Remove the current node from the open list
		open.remove(current.id);
		// Add the current node to the closed list
		closed.add(current);
	}

	return {};
}

void ofApp::pathfindingToTarget(int mouseX, int mouseY) {
	// Quantize mouse click location into the graph space
	int targetNodeIndex = quantizeToGraph(mouseX, mouseY);

	// Checking if target node index is valid
	if (targetNodeIndex != -1) {
		// Getting the start node index (current position of the boid)
		int startNodeIndex = quantizeToGraph(boid.kinematic.position.x, boid.kinematic.position.y);

		// Checking if start node index is valid
		if (startNodeIndex != -1) {
			// Using A* algorithm to find the shortest path
			path = AStar(tileGraph, startNodeIndex, targetNodeIndex, heuristic);
			cout << "Path: ";
			for (const Edge& edge : path) {
				cout << edge.source << " -> ";
			}
			cout << targetNodeIndex << endl;
			// cout << path.size() << endl;
			// Moving the boid along the computed path
			moveBoidAlongPath(path);
		}
	}
}

int ofApp::quantizeToGraph(int x, int y) {
	// Calculating grid cell size
	int numCols = ofGetWidth() / tileSize;
	float cellWidth = float(ofGetWindowWidth()) / numCols;
	int numRows = ofGetHeight() / tileSize;
	float cellHeight = float(ofGetWindowHeight()) / numRows;

	// Calculating row and column indices
	int row = int(y / cellHeight);
	int col = int(x / cellWidth);

	// Checking if row and column indices are within bounds
	if (row >= 0 && row < numRows && col >= 0 && col < numCols) {
		// Calculating node index
		int nodeIndex = row * numCols + col + 1;
		return nodeIndex;
	}
	// Returning -1 if quantized position is out of bounds
	return -1;
}

void ofApp ::moveBoidAlongPath(const vector<Edge>& path) {

	if (!path.empty()) {
		nextPathNodeIndex = 0;
		moveToNextNodeInPath();
	}
}

void ofApp::moveToNextNodeInPath() {
	if (nextPathNodeIndex >= path.size()) {
		return;
	}

	int targetNodeIndex = path[nextPathNodeIndex].sink;
	target.position.x = tileGraph.nodes[targetNodeIndex].x;
	target.position.y = tileGraph.nodes[targetNodeIndex].y;
	// cout << target.position.x << " " << target.position.y << endl;
	nextPathNodeIndex++;
}

void ofApp::reviewKinematicMotion() {
	SteeringOutput kinematicOutput = calculateSteering(boid);
	boid.kinematicUpdate(kinematicOutput);
	//Adding current position to breadcrumbs
	breadcrumbs.push_back(boid.kinematic.position);
}

void ofApp::reviewDynamicSeek() {
	SteeringOutput seekOutput = calculateSteering(boid);
	boid.update(seekOutput);
}

void ofApp::reviewDynamicArrive() {
	SteeringOutput arriveOutput = calculateSteering(boid);
	boid.update(arriveOutput);
}

void ofApp::reviewDynamicWander() {
	SteeringOutput wanderOutput = calculateSteering(boid);
	boid.update(wanderOutput);
}

void ofApp::reviewFlocking() {
	SteeringOutput leaderOutput = calculateLeaderSteering();
	monsterBoid.update(leaderOutput);
	for (int i = 0; i < numberOfFollowers; i++) {
		SteeringOutput followerOutput = calculateSteering(followerBoids[i]);
		followerBoids[i].update(followerOutput);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	// Drawing application
	ofBackground(0);
	ofSetColor(255);
	// drawUI();
	drawEnvironment();
	// drawGraph(tileGraph);
	boid.draw();
	monsterBoid.leaderDraw();

	ofSetColor(150, 150, 150);
	for (const auto& crumb : breadcrumbs) {
		// Drawing circle for breadcrumbs
		ofDrawCircle(crumb.x, crumb.y, 8);
	}

	switch (selectedBehavior) {
	case 0:
		boid.draw();
		// Drawing breadcrumbs
		ofSetColor(150, 150, 150);
		for (const auto& crumb : breadcrumbs) {
			// Drawing circle for breadcrumbs
			ofDrawCircle(crumb.x, crumb.y, 8);
		}
		break;
	case 1:
		boid.draw();
		break;
	case 2:
		boid.draw();
		break;
	case 3:
		boid.draw();
		break;
	case 4:
		monsterBoid.leaderDraw();
		for (int i = 0; i < numberOfFollowers; i++) {
			followerBoids[i].draw();
		}
		break;
	default:
		break;
	}
}

void ofApp::drawEnvironment() {
	ofColor obstacleColor = ofColor(255, 0, 0);
	ofSetColor(obstacleColor);
	for (auto& pos : obstaclePositions) {
		ofDrawRectangle(pos.x - obstacleSize / 2, pos.y - obstacleSize / 2, obstacleSize, obstacleSize);
	}
}

void ofApp::drawGraph(const Graph& graph) {
	// Drawing nodes (landmarks)
	ofSetColor(ofColor::blue); // Setting color to red for nodes

	for (const auto& node : graph.nodes) {
		// Drawing a small red square for each node
		ofDrawRectangle(node.x - 10, node.y - 10, 20, 20);
		ofDrawBitmapString(ofToString(node.id), node.x - 10, node.y - 10);
	}

	// Drawing edges (roads)
	/*for (const auto& edge : graph.edges) {
		// Getting source and sink nodes
		const Node& sourceNode = graph.nodes[edge.source];
		const Node& sinkNode = graph.nodes[edge.sink];

		// Drawing a red line connecting the two nodes
		ofSetColor(ofColor::red);
		ofDrawLine(sourceNode.x, sourceNode.y, sinkNode.x, sinkNode.y);
	}*/
}

void ofApp::drawUI() {
	ofDrawBitmapString("Press The Appropriate Number Key To Select Behavior:", 20, 20);
	ofDrawBitmapString("1. Kinematic Motion", 20, 40);
	ofDrawBitmapString("2. Dynamic Seek", 20, 60);
	ofDrawBitmapString("3. Dynamic Arrive", 20, 80);
	ofDrawBitmapString("4. Dynamic Wander", 20, 100);
	ofDrawBitmapString("5. Flocking", 20, 120);
}

SteeringOutput ofApp::KinematicMotion() {

	// Speed of boid to move along edges
	float edgeSpeed = 100.0;
	// Getting current position of boid
	Vector currentPosition = boid.kinematic.position;
	// Saving previous velocity of boid
	Vector previousVelocity = boid.kinematic.velocity;

	// Defining the points where the boid will change its direction
	Vector bottomLeft(0, ofGetHeight());
	Vector bottomRight(ofGetWidth(), ofGetHeight());
	Vector topLeft(0, 0);
	Vector topRight(ofGetWidth(), 0);

	// Calculating the distance between the boid and each edge point
	float distanceToBottomLeft = (currentPosition - bottomLeft).magnitude();
	float distanceToBottomRight = (currentPosition - bottomRight).magnitude();
	float distanceToTopLeft = (currentPosition - topLeft).magnitude();
	float distanceToTopRight = (currentPosition - topRight).magnitude();

	// If the boid is close to one of the edge points, we change its direction
	if (distanceToBottomLeft < 100) {
		return SteeringOutput(Vector(edgeSpeed, 0), 0); // Moving right along the bottom edge
	}
	else if (distanceToBottomRight < 100) {
		return SteeringOutput(Vector(0, -edgeSpeed), -PI / 2); // Moving upwards along the right edge
	}
	else if (distanceToTopRight < 100) {
		return SteeringOutput(Vector(-edgeSpeed, 0), PI); // Moving left along the top edge
	}
	else if (distanceToTopLeft < 100) {
		return SteeringOutput(Vector(0, edgeSpeed), PI / 2); // Moving downwards along the left edge
	}

	// If the boid is not close to any edge point, it continues moving with its previous velocity
	return SteeringOutput(previousVelocity, 0);
}

float ofApp::newOrientation(float current, Vector velocity) {
	if (velocity.magnitude() > 0) {
		return atan2(velocity.y, velocity.x);
	}
	else {
		return current;
	}
}

SteeringOutput ofApp::DynamicSeek(Kinematic target, float maxAcceleration) {
	// Getting the direction to the target
	Vector resultVelocity = target.position - boid.kinematic.position;

	// Giving full acceleration along this direction
	resultVelocity.normalize();
	resultVelocity *= maxAcceleration;

	// Calculating orientation as well
	//float angularAcceleration = Face(target.position);
	float angularAcceleration = newOrientation(boid.kinematic.orientation, resultVelocity);

	// Combining both for the output
	return SteeringOutput(resultVelocity, angularAcceleration);
}

SteeringOutput ofApp::DynamicEvade(Kinematic target, float maxAcceleration) {
	// Getting the direction to the target
	Vector resultVelocity = boid.kinematic.position - target.position;

	// Giving full acceleration along this direction
	resultVelocity.normalize();
	resultVelocity *= maxAcceleration;

	// Calculating orientation as well
	//float angularAcceleration = Face(target.position);
	float angularAcceleration = newOrientation(boid.kinematic.orientation, resultVelocity);

	// Combining both for the output
	return SteeringOutput(resultVelocity, angularAcceleration);
}

SteeringOutput ofApp::DynamicArrive1(Kinematic target, Boid boid, float maxAcceleration, float maxSpeed, float targetRadius, float slowRadius, float timeToTargetVelocity) {
	float targetSpeed;
	Vector direction = target.position - boid.kinematic.position;
	float distance = direction.magnitude();

	if (distance < targetRadius) {
		return NULL;
	}

	if (distance > slowRadius) {
		targetSpeed = maxSpeed;
	}

	else {
		targetSpeed = (distance - targetRadius) / slowRadius;
	}

	Vector targetVelocity = direction;
	targetVelocity.normalize();
	targetVelocity *= targetSpeed;

	Vector linearAcceleration = (targetVelocity - boid.kinematic.velocity) / timeToTargetVelocity;

	if (linearAcceleration.magnitude() > maxAcceleration) {
		linearAcceleration.normalize();
		linearAcceleration *= maxAcceleration;
	}

	float angularAcceleration = FaceForArrive(target, maxAcceleration);
	return SteeringOutput(linearAcceleration, angularAcceleration);
}

SteeringOutput ofApp::DynamicArrive2(Kinematic target, float maxAcceleration, float maxSpeed, float slowRadius, float timeToTargetVelocity) {
	float targetSpeed;
	Vector direction = target.position - boid.kinematic.position;
	float distance = direction.magnitude();

	if (distance > slowRadius) {
		targetSpeed = maxSpeed;
	}

	else {
		targetSpeed = maxSpeed * (distance / slowRadius);
	}

	Vector targetVelocity = direction;
	targetVelocity.normalize();
	targetVelocity *= targetSpeed;

	Vector linearAcceleration = (targetVelocity - boid.kinematic.velocity) / timeToTargetVelocity;

	if (linearAcceleration.magnitude() > maxAcceleration) {
		linearAcceleration.normalize();
		linearAcceleration *= maxAcceleration;
	}

	//float angularAcceleration = Face(targetVelocity);
	return SteeringOutput(linearAcceleration, 0);
}

float mapToRange(float rotation) {
	while (rotation <= -PI) {
		rotation += 2 * PI;
	}
	while (rotation > PI) {
		rotation -= 2 * PI;
	}
	return rotation;
}

SteeringOutput ofApp::Align(Kinematic target, float maxAcceleration, float maxSpeed, float targetRadius, float slowRadius, float timeToTargetRotation) {
	float targetAngle;
	Vector direction = target.orientation - boid.kinematic.orientation;
	float distance = direction.magnitude();

	if (distance < targetRadius) {
		return NULL;
	}

	if (distance > slowRadius) {
		targetAngle = maxSpeed;
	}

	else {
		targetAngle = (distance - targetRadius) / slowRadius;
	}


	float angularAcceleration = (targetAngle - boid.kinematic.rotation) / timeToTargetRotation;

	if (abs(angularAcceleration) > maxAcceleration) {
		angularAcceleration = maxAcceleration;
	}

	return SteeringOutput(Vector(), angularAcceleration);
}

float ofApp::LookWhereYoureGoing() {
	// Calculating the target to delegate to align
	Vector velocity = boid.kinematic.velocity;

	// Checking for zero direction, and make no change if so
	if (velocity.magnitude() == 0) {
		return NULL;
	}

	// Else, set the target based on the velocity
	target.orientation = atan2(velocity.y, velocity.x);

	// Delegating to Align
	SteeringOutput result = Align(target, 1000.0f, 60000.0f, 0.004f, 0.5f, 0.1f);
	return result.angular;
}

float ofApp::FaceForArrive(Kinematic target, float maxAcceleration) {
	// Calculating the target to delegate to align
	Vector direction = target.position - boid.kinematic.position;


	// Delegating to align
	float targetOrientation = atan2(direction.y, direction.x);

	float rotation = targetOrientation - boid.kinematic.orientation;

	while (rotation > PI) rotation -= 2 * PI;
	while (rotation < -PI) rotation += 2 * PI;

	float angularAcceleration = rotation;

	if (abs(angularAcceleration) > maxAcceleration) {
		angularAcceleration = maxAcceleration * (angularAcceleration > 0 ? 1 : -1);
	}

	return angularAcceleration;
}

SteeringOutput ofApp::FaceForWander(Kinematic target, float maxAcceleration) {
	// Calculating the target to delegate to align
	Vector direction = target.position - boid.kinematic.position;


	// Delegating to align
	float targetOrientation = atan2(direction.y, direction.x);

	float rotation = targetOrientation - boid.kinematic.orientation;

	while (rotation > PI) rotation -= 2 * PI;
	while (rotation < -PI) rotation += 2 * PI;

	float angularAcceleration = rotation;

	if (abs(angularAcceleration) > maxAcceleration) {
		angularAcceleration = maxAcceleration * (angularAcceleration > 0 ? 1 : -1);
	}

	return SteeringOutput(Vector(0, 0), angularAcceleration);
}

float ofApp::randomBinomial() {
	return (ofRandom(-1, 1) - ofRandom(-1, 1));
}

Vector asVector(float angle) {
	return Vector(cos(angle), sin(angle));
}

SteeringOutput ofApp::Wander1(Boid boid, float wanderOffset, float wanderRadius, float wanderRate, float wanderOrientation, float maxAcceleration) {
	wanderOrientation += randomBinomial() * wanderRate;

	float targetOrientation = wanderOrientation + boid.kinematic.orientation;

	Vector target = boid.kinematic.position + asVector(boid.kinematic.orientation) * wanderOffset;

	target = target + Vector(cos(targetOrientation), sin(targetOrientation)) * wanderRadius;

	SteeringOutput result = FaceForWander(target, maxAcceleration);

	result.linear = asVector(boid.kinematic.orientation) * maxAcceleration;

	return result;
}

SteeringOutput ofApp::Wander2(float wanderRadius, float wanderDistance, float wanderAngle, float maxAcceleration) {
	// Caluclating the wander circle center
	Vector normalizedVector = boid.kinematic.velocity;
	normalizedVector.normalize();
	Vector wanderCenter = normalizedVector * wanderDistance;

	// Calculating the displacement vector within the wander circle
	Vector displacement = Vector(cos(wanderAngle), sin(wanderAngle)) * wanderRadius;

	// Setting the target position to be the sum of wanderCenter and displacement
	Vector targetPosition = boid.kinematic.position + wanderCenter + displacement;

	// Seeking the target position
	SteeringOutput seekSteering = DynamicSeek(targetPosition, maxAcceleration);

	// Calculating the 'look where you're going' behavior
	float targetOrientation = atan2(boid.kinematic.velocity.y, boid.kinematic.velocity.x);
	float rotation = targetOrientation - boid.kinematic.orientation;

	// Ensuring the rotation is in range [-pi, pi]
	while (rotation > PI) rotation -= 2 * PI;
	while (rotation < -PI) rotation += 2 * PI;

	// Calculating the angular acceleration to align with the velocity direction
	float angularAcceleration = rotation;

	// Limiting the angular acceleration to the maximum allowed
	if (abs(angularAcceleration) > maxAcceleration) {
		angularAcceleration = maxAcceleration * (angularAcceleration > 0 ? 1 : -1);
	}

	// Returning the combined steering output
	return SteeringOutput(seekSteering.linear, angularAcceleration);
}

SteeringOutput ofApp::Separation(Boid boid, float maxAcceleration, vector<Boid> targets, float threshold, float decay) {
	SteeringOutput result;

	for (const auto& t : targets) {
		Vector direction = t.kinematic.position - boid.kinematic.position;
		float distance = direction.magnitude();

		if (distance < threshold) {
			float strength = min((decay / (distance * distance)), maxAcceleration);

			direction.normalize();
			result.linear = result.linear + (direction * strength);
		}
	}

	Vector direction = monsterBoid.kinematic.position - boid.kinematic.position;
	float distance = direction.magnitude();
	if (distance < threshold) {
		float strength = min((decay / (distance * distance)), maxAcceleration);

		direction.normalize();
		result.linear = result.linear + (direction * strength);
	}

	return result;
}

SteeringOutput ofApp::VelocityMatch(Boid boid) {
	SteeringOutput result;

	Vector averageVelocity;

	float totalWeight = 0.0f;
	for (int i = 0; i < numberOfFollowers; i++) {
		float weight = 1.0f;
		totalWeight += weight;
		averageVelocity = averageVelocity + (followerBoids[i].kinematic.velocity * weight);
	}

	averageVelocity = averageVelocity + (monsterBoid.kinematic.velocity * 100.0f);
	totalWeight += 100.0f;
	averageVelocity = averageVelocity / totalWeight;

	result.linear = (averageVelocity - boid.kinematic.velocity);
	return result;
}

SteeringOutput ofApp::Flocking(Boid& follower) {
	SteeringOutput combinedSteering;

	SteeringOutput separationSteering = Separation(follower, 10.0f, followerBoids, 400.0f, 20.0f);
	combinedSteering.linear = combinedSteering.linear + separationSteering.linear;

	Kinematic centreOfMass;
	float totalMass = 0.0f;
	for (int i = 0; i < numberOfFollowers; i++) {
		float mass = 1.0f;
		totalMass += mass;
		centreOfMass.position = centreOfMass.position + (boid.kinematic.position * mass);
	}

	centreOfMass.position = centreOfMass.position + (monsterBoid.kinematic.position * 100.0f);
	totalMass += 100.0f;
	centreOfMass.position = centreOfMass.position / totalMass;
	SteeringOutput arriveSteering = DynamicArrive1(centreOfMass, follower, 2.0f, 15.0f, 5.0f, 100.0f, 0.1f);
	arriveSteering.linear = arriveSteering.linear * 0.8f;
	combinedSteering.angular = combinedSteering.angular + arriveSteering.angular;
	combinedSteering.linear = combinedSteering.linear + arriveSteering.linear;

	SteeringOutput velocityMatchSteering = VelocityMatch(follower);
	velocityMatchSteering.linear = velocityMatchSteering.linear * 0.6f;
	combinedSteering.linear = combinedSteering.linear + velocityMatchSteering.linear;

	return combinedSteering;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	// Change selected behavior based on user input
	switch (key) {
	case '1':
		selectedBehavior = 0;
		break;
	case '2':
		selectedBehavior = 1;
		break;
	case '3':
		selectedBehavior = 2;
		break;
	case '4':
		selectedBehavior = 3;
		break;
	case '5':
		selectedBehavior = 4;
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	pathfindingToTarget(x, y);
	// target.position.set(x, y);
	//target.orientation = 0.0f;
	// boid.kinematic.velocity.set(0, 0);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
