/*author Varun Verlencar
* Worcester Polytechnic Insitute
Last update: 23 March 2016
*/

#pragma once

#include <vector>
#include <algorithm>

const double pi = 3.14;
bool goalflag = false;

///////////////////// RRTNode class with Node configuration and its parent /////////////////////

class RRTNode {
private:
	std::vector<float> _config;
	RRTNode* _parent;

public:
	RRTNode(const std::vector<float> &q, RRTNode* prevnode = NULL){
		_config.assign(q.begin(),q.end());
		this->_parent = prevnode;
	}

	RRTNode(RRTNode* Node)	{
		_config.assign(Node->getConfig().begin(),Node->getConfig().end());
		this->_parent = Node->getParent();
	}

	void copyNode(RRTNode* newNode)	{
		
		_config.assign(newNode->getConfig().begin(),newNode->getConfig().end());
		this->_parent = newNode->getParent();
	}

	virtual ~RRTNode() {
		// delete _config;
		// delete _parent;
	}

	RRTNode* getParent() {return _parent;}

	const std::vector<float> &getConfig() {return _config;}

	//for Bi-RRT
	// int depth() const 
	// {
	// 	int n = 0;
	// 	for (RRTNode *ancestor = _parent; ancestor != NULL; ancestor = ancestor->_parent) {
	// 	n++;
	// 	}
	// return n;
	// }
};

///////////////////// NodeTree class with all Nodes /////////////////////

class NodeTree {
private:
	std::vector<RRTNode*> _nodes;
	std::vector<float> _goalconfig;
	float _stepSize;
	float _goalBias;
	int _maxIterations;
	unsigned int _numNodes;
	float _errorfactor;
	bool _goalflag;

public:
	NodeTree()
	{
		//default values
		_numNodes = 1;
		setStepSize(0.25);
		// setGoalBias(0.26); // value between 0 and 1
		setMaxIterations(10000);
		setErrorFactor(1);
	}

	NodeTree(RRTNode* start)
	{
		_nodes.push_back(start);
		_numNodes = 1;
		setStepSize(0.25);
		// setGoalBias(0.26); // value between 0 and 1
		setMaxIterations(10000);
		setErrorFactor(1);
	}

	virtual ~NodeTree() {
		// delete _nodes;
		// delete _goalconfig;
		// delete _stepSize;
		// delete _goalBias;
		// delete _maxIterations;
		// delete _numNodes;
		// delete _errorfactor;
		// delete _goalflag;
	}

	//add nodes to the tree
	void addNode(RRTNode* _newNode){
		_nodes.push_back(_newNode);
		++_numNodes;
	}

	//delete nodes from the tree
	void deleteNode(unsigned int index){
		_nodes.pop_back();
		--_numNodes;
	}

	//get path from root to node at index
	NodeTree* getPath(unsigned int index){
		RRTNode* node = _nodes[index];
		NodeTree* path = new NodeTree(node);

		while(node->getParent() != NULL)
		{
			path->addNode(node); // insert 
			node = node->getParent();
		}
		path->addNode(node); 
		// std::reverse(path->_nodes.begin(),path->_nodes.end());
		return path;
	}


	RRTNode* getNodes(unsigned int index) {return _nodes[index];}

	//set goal bias
	void setGoalBias(float goalb){
		if (goalb < 0 || goalb > 1) {
			throw std::invalid_argument("The goal bias must be a number between 0.0 and 1.0");
		}
		_goalBias = goalb;
	}


	void addNodeTree(NodeTree* intermediateTree)
	{
		for(unsigned int i=0;i<intermediateTree->sizeNodes();i++)
		{
			_nodes.push_back(intermediateTree->getNodes(i));
			++_numNodes;
		}
	}

	///////////////////// Find the nearest node from the tree to current node /////////////////////

	RRTNode* nearestNeighbour(RRTNode* randomNode)
	{
		// std::cout<<"Searching Nearest Neighbour...:"<<std::endl;
		int dist = -1;
		int nearestindex = 0;
		
		for (unsigned int i = 0; i<_nodes.size();++i) {
			float currentdist = getNearestDistance(_nodes[i]->getConfig(), randomNode->getConfig());
			// std::cout<<"checking Node...:"<<i<<std::endl;
			if (dist < 0 || currentdist < dist) {
				dist = currentdist;
				nearestindex = i;
			}
		}

		// RRTNode* p = _nodes[nearestindex];

		// std::vector<float> nn(_nodes[nearestindex]->getConfig().begin(),_nodes[nearestindex]->getConfig().end());
		std::cout<<"Nearest node at:"<<nearestindex<<" Distance of:"<<dist<<std::endl;
		// std::cout<<"Nearest found:["<<nn[0]<<","<<nn[1]<<","<<nn[2]<<","<<nn[3]<<","<<nn[4]<<","<<nn[5]<<","<<nn[6]<<"]"<<std::endl;

		// std::vector<float>::const_iterator it;
		// for(it=_nodes[nearestindex]->getConfig().begin(); it!=_nodes[nearestindex]->getConfig().end(); ++it){
		// 	std::cout<<"Nearest found:"<<(*it)<<std::endl;
		// }
		return _nodes[nearestindex];
	}

	///////////////////// Find euclidean distance between given configuration and tree nodes /////////////////////

	float getDistance(const std::vector<float> &existingNodeConfig, const std::vector<float> &newconfig)
	{
		//euclidean distance
		// std::cout<<"Entered getDistance..."<<std::endl;
		float tempdist = 0;
		for (unsigned int i = 0;i<newconfig.size();++i)
		{
			tempdist += pow(newconfig[i] - existingNodeConfig[i],2);
		}
		tempdist = std::sqrt(tempdist);
		return tempdist;
	}

	///////////////////// Find weighted euclidean distance between given configuration and tree nodes /////////////////////

	float getNearestDistance(const std::vector<float> &existingNodeConfig, const std::vector<float> &newconfig)
	{
		//euclidean distance wieghted
		// std::cout<<"Entered getNearestDistance..."<<std::endl;
		float tempdist = 0;
		float w[7] = {800.0, 600.0, 300.0, 100.0, 0.0, 5.0, 0.0};
		// float w[7] = {1,1,1,1,1,1,1};
		for (unsigned int i = 0;i<newconfig.size();++i)
		{
			tempdist += w[i]*pow((newconfig[i] - existingNodeConfig[i]),2);
		}
		tempdist = std::sqrt(tempdist);
		return tempdist;
	}

	///////////////////// Sample a new random node /////////////////////

	RRTNode* getRamdomSample(std::vector<float> upper,std::vector<float> lower,RRTNode* goalNode)
	{	
		// std::cout<<"Entered Sampling..."<<std::endl;
		std::vector<float> sampleconfig;
		// NodeTree* p = new NodeTree();

		for (int i=0;i<7;i++){		
			sampleconfig.push_back(0);
		}

		float r = rand()/(float)RAND_MAX; //  r is between 0 and 1 since we normalize it
		std::cout<<"random r:"<<r<<std::endl;

		if (r < 0.26) {//p->goalBias()
			std::cout<<"Goal as Sample...:"<<std::endl<<std::endl;
			// p->setgoalflag(true);
			goalflag=true;
			return goalNode;
		}

		else {
			std::cout<<"Getting Random Sample...:"<<std::endl<<std::endl;
			// p->setgoalflag(false);
			goalflag =false;
			for (int i = 0; i < 7; ++i){
				sampleconfig[i]=((rand() * (upper[i]-lower[i]))/(float)RAND_MAX) + lower[i];
			}	

		RRTNode* RandNode = new RRTNode(sampleconfig);
		return RandNode;
		}
	}

	///////////////////// Check for collision /////////////////////
			
	bool checkifCollision(const std::vector<float> &sampleconfig,OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot)
	{
		std::vector<OpenRAVE::dReal> config(sampleconfig.begin(),sampleconfig.end());
		robot->SetActiveDOFValues(config);
		bool collision = env->CheckCollision(robot);
		bool selfcollision = robot->CheckSelfCollision();
		
		// float lolimit={-0.564602,-0.3536,-2.12131,-0.650001,-10000,-2.00001,-10000};
		// float uplimit ={2.13539,1.2963,-0.15,3.75,10000,-0.1,1000};

		if(collision||selfcollision)
			std::cout<<"Collision!"<<std::endl;
		return (collision||selfcollision);
	}

	///////////////////// Connect sampled node to nearest node in tree /////////////////////

	NodeTree* connectNodes(RRTNode* sampledNode, RRTNode* nearestNode,RRTNode* goalNode, float stepsize,OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot)
	{
		std::vector<float> goal = goalNode->getConfig();
		std::cout<<"Connecting..."<<std::endl;

		std::vector<float> targetConfig(sampledNode->getConfig().begin(),sampledNode->getConfig().end());
		std::vector<float> unitConfig;
		NodeTree* intermediateTree;
		// NodeTree* q = new NodeTree();

		std::cout<<"Entered connect"<<std::endl<<std::endl;
		bool flag = true; // for first node additions
		std::vector<float> nearestNodeConfig(nearestNode->getConfig().begin(),nearestNode->getConfig().end());
		RRTNode* currentNode;
		RRTNode* prevNode = nearestNode;

		for (int itr=0; itr<10000; ++itr){ // avoid infinite loops, change as required
			std::cout<<std::endl<<"Enter connectloop... iteration"<<itr<<std::endl;
			std::vector<float>::const_iterator it;
			std::vector<float> prevConfig(nearestNodeConfig.begin(),nearestNodeConfig.end());
			
			float ndist = getNearestDistance(targetConfig,nearestNodeConfig);

			std::cout<<"Sampledistance:"<<ndist<<std::endl;

			if (ndist > stepsize){
				std::cout<<"Step:"<<itr<<std::endl;

				for(int i = 0; i < 7; ++i){
					unitConfig.push_back((targetConfig[i] - nearestNodeConfig[i])/ndist);
					nearestNodeConfig[i] += stepsize * unitConfig[i]; // find unit vector
				}
				// for(it=nearestNodeConfig.begin(); it!=nearestNodeConfig.end(); ++it){
				// 	std::cout<<"New nearestNode:"<<(*it)<<std::endl;
				// }
				

				if(getNearestDistance(nearestNodeConfig,prevConfig)){ // check if moved to new location, remove if fails
					if(!checkifCollision(nearestNodeConfig,env,robot)){
						currentNode  = new RRTNode(nearestNodeConfig, prevNode);
						prevNode = currentNode;	//check this
						// std::cout<<"prev-> curr:"<<std::endl;
						
						//print contents of added node
						std::cout<<"...New node created:"<<std::endl;
						for(it=currentNode->getConfig().begin(); it!=currentNode->getConfig().end(); ++it){
							std::cout<<(*it)<<std::endl;
						}

						// for(it=currentNode->getParent()->getConfig().begin(); it!=currentNode->getParent()->getConfig().end(); ++it){
						// 	std::cout<<"currentNodeparent:"<<(*it)<<std::endl;
						// }

						// for(it=prevNode->getConfig().begin(); it!=prevNode->getConfig().end(); ++it){
						// 	std::cout<<"prevNode:"<<(*it)<<std::endl;
						// }

						if (flag)
						{
							intermediateTree = new NodeTree(currentNode);
							flag = false;
						}
						else
						{
							intermediateTree->addNode(currentNode);
						}
						std::cout<<"...New node Added:"<<std::endl;
					}
					else
					{
						// std::cout<<"in dist >step"<<std::endl;
						if (flag)
						{
							intermediateTree = new NodeTree();
							flag = false;
						}			
						break;
					 //check if NULL
					}
					
				}
				// if new sampled node is the same as previous
				else
				{	
					// std::cout<<"Not moved"<<std::endl;
					if (flag)
						{
							intermediateTree = new NodeTree();
							flag = false;
						}
					break;
				}
			}
			// if the distance within the goal threashold range
			else if(ndist <= 1.0) //q->errorfactor()
			{	
				std::cout<<std::endl<<"..Step :"<<itr<<"->inside epsilon "<<std::endl;
				//if the distance is very close
				if(!checkifCollision(nearestNodeConfig,env,robot))
				{
					std::cout<<"...Entered Goal Zone..."<<std::endl;
					if (goalflag)	// if sampled node was goal 
					{	
						currentNode  = new RRTNode(goal, prevNode);
						if (flag)
						{
							intermediateTree = new NodeTree(currentNode);
							flag = false;
						}
						else
						{
							intermediateTree->addNode(currentNode);
							
						}
						std::cout<<"............Goal reached..."<<std::endl;
						break;
					
					}
					// else sampled node was reached and added 
					else
					{	currentNode  = new RRTNode(nearestNodeConfig, prevNode);
						if (flag)
						{
							intermediateTree = new NodeTree(currentNode);
							flag = false;
						}
						else
						{
							intermediateTree->addNode(currentNode);
						}
						std::cout<<"............Sample reached..."<<std::endl;
						break;
					}					
				}
				else
				{	
					// std::cout<<"in dist<step"<<std::endl;
					if (flag)
						{
							intermediateTree = new NodeTree();
							flag = false;
						}
					break;
				}

			}
			else {
				std::cout<<"(o_o)(o_o)(o_o)(o_o)(o_o).."<<std::endl;
			}
		}
		std::cout<<"Tree returned"<<intermediateTree->sizeNodes()<<std::endl;
		return intermediateTree;
	}	

	///////////////////// Grow RRT tree /////////////////////

	NodeTree* rrtgrow(const std::vector<float> &start,const std::vector<float> &goal,float goalbias,std::vector<float> upper,std::vector<float> lower,OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot)
	{
		RRTNode* startNode = new RRTNode(start);
		RRTNode* goalNode = new RRTNode(goal);
		setgoalConfig(goal);
		setGoalBias(goalbias);
		NodeTree* initPath = new NodeTree(startNode);
		
		NodeTree* path= new NodeTree();
		NodeTree* finalPath=new NodeTree();
		RRTNode* nearestNode = NULL;
		
		// initPath->setgoalflag(false);
		goalflag = false;

		std::vector<float>::const_iterator it;
		std::cout<<std::endl<<"Given:"<<std::endl<<"Goal:"<<goal[0]<<","<<goal[1]<<","<<goal[2]<<","<<goal[3]<<","<<goal[4]<<","<<goal[5]<<","<<goal[6]<<std::endl;
		std::cout<<std::endl<<"Start:"<<start[0]<<","<<start[1]<<","<<start[2]<<","<<start[3]<<","<<start[4]<<","<<start[5]<<","<<start[6]<<std::endl;


		for (int i=0; i<10000;++i){
			std::cout<<std::endl<<"RRT-iteration"<<i<<std::endl;
			
			RRTNode* sampledNode = initPath->getRamdomSample(upper,lower,goalNode);
			
			if(!checkifCollision(sampledNode->getConfig(),env,robot)){
				nearestNode = initPath->nearestNeighbour(sampledNode);

				std::vector<float> nn(nearestNode->getConfig().begin(),nearestNode->getConfig().end());
				std::cout<<"Nearest found:["<<nn[0]<<","<<nn[1]<<","<<nn[2]<<","<<nn[3]<<","<<nn[4]<<","<<nn[5]<<","<<nn[6]<<"]"<<std::endl;
				// std::cout<<"Nearest Node found..."<<std::endl;
				
				path = initPath->connectNodes(sampledNode, nearestNode, goalNode,initPath->stepSize(),env,robot);
				
				
			}
			else
				continue;
						 
			if(path->sizeNodes()==1){
				std::cout<<"restart subpath.."<<std::endl;
			 	continue;
			}
			else if(getNearestDistance(path->lastNode()->getConfig(),goal)==0){
				// std::cout<<"found found..."<<std::endl;
				initPath->addNodeTree(path);
				std::cout<<"...Final..."<<std::endl;
				
				for(it=path->lastNode()->getConfig().begin(); it!=path->lastNode()->getConfig().end(); ++it){
					std::cout<<"final Node:"<<(*it)<<std::endl;
				}
				
				break;
			}
			else{
				initPath->addNodeTree(path);
				std::cout<<"Intermediate Tree..."<<std::endl;
				// finalPath = initPath->getPath(initPath->sizeNodes()-1);
			}			
		}
		std::cout<<" Path found..."<<std::endl;

		finalPath = initPath->getPath(initPath->sizeNodes()-1);
		std::cout<<" initPath Size..."<<initPath->sizeNodes()<<std::endl;

		std::cout<<" FinalPath Size..."<<finalPath->sizeNodes()<<std::endl;
		
		return finalPath;
	}

// 	//////////////////////// Shortcut smoothing to smooth the rrt path ////////////////////

// vector< vector<dReal> > smoothpath(vector< vector<dReal> > path_ruggard, float stepsize, float* weight, EnvironmentBasePtr& env, RobotBasePtr& robot)
// {
// 	int hi, ran1, ran2, p1, p2, coll;
// 	std::vector<float>  v1, v2, inter;
// 	vector< vector<dReal> > bypass;
// 	vector< vector<dReal> >::iterator it;


// 	for(int i=0; i<200; i++){
// 		hi =path_ruggard.size();
// 		ran1 = rand()%hi;
// 		ran2 = rand()%hi;
// 		p1=min(ran1,ran2);
// 		p2=max(ran1,ran2);
// 		if(p1 != p2){
// 			v1 = path_ruggard[p1];
// 			v2 = path_ruggard[p2];
// 			inter = v1;
// 			bypass.push_back(v1);
// 			it = path_ruggard.begin();

// 			while(1){
// 				inter = extend(inter, v2, stepsize, weight);
// 				robot->SetActiveDOFValues(inter);
// 				coll = env->CheckCollision(robot) || robot->CheckSelfCollision();
// 				if(coll == 1){
// 					break;
// 				}
// 				else{
// 					bypass.push_back(inter);
// 					if(distance(inter, v2, weight) < stepsize){
// 						break;
// 					}
// 				}
// 			}
// 			if(coll == 0){
// 				path_ruggard.erase(path_ruggard.begin()+p1, path_ruggard.begin()+p2);
// 				path_ruggard.insert(it+p1, bypass.begin(), bypass.end());
// 			}
// 			bypass.clear();
// 		}
// 	}	
// 	return path_ruggard;
// }

	///////////////////// Useful finctions /////////////////////

	const std::vector<float> goalConfig(){return _goalconfig;}

	const std::vector<RRTNode*> allNodes() {return _nodes;}

	unsigned int sizeNodes(){return _numNodes;}

	void setgoalConfig(const std::vector<float> &goalConfig) {_goalconfig = goalConfig;}

	void setgoalflag(bool f=false) {_goalflag = f;}

	bool goalFlag() {return _goalflag;}

	float stepSize() const {return _stepSize;}

	void setStepSize(float stepSize = 0.25) {_stepSize = stepSize;}

	float errorfactor() const {return _errorfactor;}

	void setErrorFactor(float err = 1) {_errorfactor = err;}

	float goalBias() const {return _goalBias;}

	int maxIterations() const {return _maxIterations;}

	void setMaxIterations(int itr = 1000) {_maxIterations = itr;}

	RRTNode* rootNode() const {
		if (_nodes.empty()) 
			return NULL;

	return _nodes.front();
	}


	RRTNode* lastNode() const {
		if (_nodes.empty()) 
			return NULL;

	return _nodes.back();
	}

};


//get nodes
