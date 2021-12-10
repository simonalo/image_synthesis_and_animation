#include "joint.h"
#include <QtGui/QMatrix4x4>

using namespace std;

Joint* Joint::createFromFile(std::string fileName) {
	Joint* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	
			Joint::parseHierarchy(inputfile, buf) 

			// Parse joints
			Joint::parseJoint(inputfile, buf) 
			// Parse motion
		}
		inputfile.close();
	} else {
		std::cerr << "Failed to load the file " << fileName.data() << std::endl;
		fflush(stdout);
	}

	cout << "file loaded" << endl;

	return root;
}

void Joint::animate(int iframe) 
{
	// Update dofs :
	_curTx = 0; _curTy = 0; _curTz = 0;
	_curRx = 0; _curRy = 0; _curRz = 0;
	for (unsigned int idof = 0 ; idof < _dofs.size() ; idof++) {
		if(!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
	}	
	// Animate children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->animate(iframe);
	}
}


void Joint::nbDofs() {
	if (_dofs.empty()) return;

	double tol = 1e-4;

	int nbDofsR = -1;

	// TODO :
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}

void Joint::parseHierarchy(ifstream& file, string& buf) {
	file >> buf;

	if (buf != kHierarchy) {
		cerr << "Error : Can't find HIERARCHY keyword." << endl;
		exit(EXIT_FAILURE);
	}
}

void Joint::parseJoint(ifstream& file, string& buf) {
	// Parse root
	file >> buf;

	if (buf != kRoot) {
		cerr << "Error : Can't find ROOT keyword." << endl;
		exit(EXIT_FAILURE);
	}
	
	// Creating root Joint
	Joint* root = new Joint();

	// Parse offset
	Joint::parseOffset(file, buf, root);

	// Parse channel
	Joint::parseChannels(file, buf, root);

	// Parse childs
	Joint* parent;
	Joint* current = root;
	
	while (buf!=kMotion) {
		file >> buf;
		if (buf == kJoint) {
			parent = current;
			current = new Joint();
			Joint::parseOffset(file, buf, name, current, parent);
			Joint::parseChannels(file, buf, current);
		}
		else if (buf == kEnd) {
			// Last joint : we parse only offset
			parent = current;
			current = new Joint();
			Joint::parseOffset(file, buf, name, current);
		}
		else if (buf == kbrackC) {
			// We go up in the tree
			current = current->_parent;
		}
		else if (buf == kMotion) {
			// Next part (motion)
			break;
		}
		else {
			cerr << "Error : keyword " << buf << "not handled." << endl;
			exit (EXIT_FAILURE);
		}
	}
}

void Joint::parseOffset(ifstream& file, string& buf, Joint* joint, Joint* parent) {
	// Parse name of joint before offset
	string name;
	file >> name;

	file >> buf; // Parse {
	
	file >> buf;
	if (buf != kOffset) {
		cerr << "Error : Can't find OFFSET keyword." << endl;
		exit(EXIT_FAILURE);
	}

	double offset_x; double offset_y; double offset_z;
	try {
		file >> offset_x >> offset_y >> offset_z;
	}
	catch(...) {
		cerr << "Error : Can't read values of offset."<<endl;
		exit (EXIT_FAILURE);
	}

	joint = *Joint::create(name, offset_x, offset_y, offset_z, parent);
}

void Joint::parseChannels(ifstream& file, string& buf, Joint* joint) {
	file >> buf;
	if (buf != kChannels) {
		cerr << "Error : Can't find CHANNELS keyword." << endl;
		exit(EXIT_FAILURE);
	}

	file >> buf;
	int nb_channels = stoi(buf);

	// Add new animation curve (euler angle for rotation are given later in Motion)
	for (int i = 0; i < nb_channels; i++) {
		file >> buf;

		AnimCurve currAnim;
		currAnim.name = buf;

		current->_dofs.push_back(currAnim);
	}
}