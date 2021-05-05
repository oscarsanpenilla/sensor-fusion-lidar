/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		// if root is NULL make node root node and return
		std::cout << "Point:" << point.at(0) << "," << point.at(1) << "\n";
		if (root == NULL)
		{
			std::cout << "Added root Node\n";
			root = new Node(point, 0);
			return;
		}

		// while node is not NULL
		int nodeCount = 0, dimension = 2;
		Node *currentNode = root;
		while (true)
		{
			// Increase node count
			++nodeCount;
			// Calculate the criteria for insertion node_count%dimension 0->x, 1->y, 2->z
			int criteria = nodeCount % dimension == 1 ? 0 : 1;
			std::cout << "nodeCount:" << nodeCount << " criteria:" << criteria << " CurrentNode " << currentNode->point.at(0) << "," << currentNode->point.at(1) << "\n";

			// Compare the point dimension based in the criteria
			if (criteria == 0)
				std::cout << "Comparing X component\n";
			else
				std::cout << "Comparing Y component\n";

			if (point.at(criteria) >= currentNode->point.at(criteria))
			{
				std::cout << "Right side\n";
				// If the next node is null, insert the new node and break
				if (currentNode->right == NULL)
				{
					currentNode->right = new Node(point, nodeCount);
					break;
				}
				else
					currentNode = currentNode->right;
			}
			else
			{
				std::cout << "Left side\n";
				if (currentNode->left == NULL)
				{
					currentNode->left = new Node(point, nodeCount);
					break;
				}
				else
					currentNode = currentNode->left;
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
