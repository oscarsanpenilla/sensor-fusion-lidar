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

	void insertHelper(Node **node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			// condition = depth%dimension
			uint condition = depth % 2;
            if ((*node)->point[condition] > point[condition])
                insertHelper(&(*node)->left, depth + 1, point, id);
			else
                insertHelper(&(*node)->right, depth + 1, point, id);
		}
	};

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function creates a new node and place correctly with in the root
		int depth = 0;
		insertHelper(&root, depth, point, id);
	}

    void searchHelper(Node** node, std::vector<float> target, float distanceTol, std::vector<int> &ids, int depth)
    {
        if ((*node) == NULL)
            return;

        float xt = target[0];
        float yt = target[1];
        float xc = (*node)->point[0];
        float yc = (*node)->point[1];

        // Check if the current node is inside the target box
        bool isInsideBox = fabs(xt-xc)<= distanceTol && fabs(yt-yc)<= distanceTol;
        if (isInsideBox)
        {
            // If so, calculate the distance between the node and the target
            float targetDist = sqrt((pow(xt-xc,2) + (pow(yt-yc,2))));
            // If distance <= distanceTol, add node id to list of nearby points
            if (targetDist <= distanceTol)
                ids.push_back((*node)->id);
            // If not, the node isn't a neighbor
        }

        // If the box crosses over the current node division compare the next node
        uint criteria = depth % 2;
        if ((*node)->point[criteria] > (target[criteria] - distanceTol))
                searchHelper(&((*node)->left), target, distanceTol, ids, depth + 1);
        if ((*node)->point[criteria] <= (target[criteria] + distanceTol))
                searchHelper(&((*node)->right), target, distanceTol, ids, depth +1);
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(&root, target, distanceTol, ids, 0);
		return ids;
	}
};
