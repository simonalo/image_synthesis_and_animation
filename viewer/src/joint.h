#ifndef _JOINT_H_
#define _JOINT_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "../../trimesh2/include/Box.h"
#include <QMatrix4x4>
#include <QVector3D>

// Keywords usefull for parser
const std::string kHierarchy = "HIERARCHY";
const std::string kRoot = "ROOT";
const std::string kOffset = "OFFSET";
const std::string kChannels = "CHANNELS";
const std::string kJoint = "JOINT";
const std::string kEnd = "End";
const std::string kMotion = "MOTION";
const std::string kbrackO = "{";
const std::string kbrackC = "}";

using namespace std;

class AnimCurve {
public :
	AnimCurve() {};
	~AnimCurve() {
		_values.clear();
	}
public :
	std::string name;					// name of dof
	std::vector<double> _values;		// different keyframes = animation curve
	int size(){return _values.size(); }
};


enum RotateOrder {roXYZ=0, roYZX, roZXY, roXZY, roYXZ, roZYX};

class Joint {
public :
	static std::vector<string> list_names;
	static float FRAME_TIME;
	static int max_id;
	int id;
	std::string _name;					// name of joint
	double _offX;						// initial offset in X
	double _offY;						// initial offset in Y
	double _offZ;						// initial offset in Z
	std::vector<AnimCurve> _dofs;		// keyframes : _animCurves[i][f] = i-th dof at frame f;
	double _curTx;						// current value of translation on X
	double _curTy;						// current value of translation on Y
	double _curTz;						// current value of translation on Z
	double _curRx;						// current value of rotation about X (deg)
	double _curRy;						// current value of rotation about Y (deg)
	double _curRz;						// current value of rotation about Z (deg)
	int _rorder;						// order of euler angles to reconstruct rotation
	std::vector<Joint*> _children;		// children of the current joint
	Joint* _parent; 					// parent of the current joint


public :
	// Constructor :
	Joint();
	// Destructor :
	~Joint();
	static int findIndexOfJoint(string name);

	void getTransformationMatrices(std::vector<QMatrix4x4>& bindedMatrices, std::vector<QMatrix4x4>& transformMatrices); // A appliquer sur le root

    void getChildTransformationMatrices(std::vector<QMatrix4x4>& bindedMatrices,std::vector<QMatrix4x4>& transformMatrices,
                                        QMatrix4x4 parentPosition, QMatrix4x4 parentRotation);


	// Create from data :
	static Joint* create(Joint* child, std::string name, double offX, double offY, double offZ, Joint* parent) {
		child->_name = name;
		child->_offX = offX;
		child->_offY = offY;
		child->_offZ = offZ;
		child->_curTx = 0;
		child->_curTy = 0;
		child->_curTz = 0;
		child->_curRx = 0;
		child->_curRy = 0;
		child->_curRz = 0;
		child->_parent = parent;
		if(parent != nullptr) {
			parent->_children.push_back(child);
		}
		return child;
	}

	// Load from file (.bvh) :	
	static Joint* createFromFile(std::string fileName);

	void animate(int iframe=0);

	static void parseHierarchy(ifstream& file, string& buf);
	static Joint* parseJoint(ifstream& file, string& buf);
	static void parseOffset(ifstream& file, string& buf, Joint* joint, Joint* parent);
	static void parseChannels(ifstream& file, string& buf, Joint* joint);
	static void parseMotion(ifstream& file, string& buf, Joint* joint);
	static void parseFrame(ifstream& file, string& buf, Joint* joint);
	// Analysis of degrees of freedom :
	void nbDofs();

	vector<trimesh::point> exportPositions();

	void exportPositions(QMatrix4x4& transform, vector<trimesh::point>& positions);

	void exportChildPositions(QMatrix4x4& matriceTransformation,
			QVector3D& positionRoot, vector<trimesh::point> &positions);

    vector<trimesh::point> exportMiddleArticulations();

    void exportChildMiddleArticulations(QMatrix4x4& matriceTransformation,
            QVector3D& positionRoot, vector<trimesh::point> &positions);


};


#endif