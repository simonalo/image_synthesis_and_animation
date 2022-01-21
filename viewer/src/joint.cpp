#include "joint.h"
#include <QtGui/QMatrix4x4>

using namespace std;

int Joint::max_id = 0;
std::vector<string> Joint::list_names;

Joint::Joint(){
	id = max_id;
	max_id++;
}

Joint* Joint::createFromFile(std::string fileName) {
	Joint::max_id = 0;
	Joint* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	

    	
			Joint::parseHierarchy(inputfile, buf) ;
			
			// Parse joints
			Joint::parseJoint(inputfile, buf);
			
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
	Joint::parseOffset(file, buf, root, nullptr);

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
			Joint::parseOffset(file, buf, current, parent);
			Joint::parseChannels(file, buf, current);
		}
		else if (buf == kEnd) {
			// Last joint : we parse only offset
			parent = current;
			current = new Joint();
			Joint::parseOffset(file, buf, current, parent);
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
			cerr << "Error : keyword " << buf << " not handled." << endl;
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

	joint = Joint::create(name, offset_x, offset_y, offset_z, parent);
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

		joint->_dofs.push_back(currAnim);
	}
}

int Joint::findIndexOfJoint(string name){
	auto it = std::find(Joint::list_names.begin(), Joint::list_names.end(), name);
	if (it == Joint::list_names.end()) {
	  return -1;
	}
  	auto index = std::distance(Joint::list_names.begin(), it);
	return (int)index;
}


void Joint::getTransformationMatrices(std::vector<QMatrix4x4>& bindedMatrices, std::vector<QMatrix4x4>& transformMatrices){
    QMatrix4x4 positionOffset;
    QMatrix4x4 rotation;
    getChildTransformationMatrices(bindedMatrices, transformMatrices, positionOffset, rotation);
}

void Joint::getChildTransformationMatrices(std::vector<QMatrix4x4>& bindedMatrices,std::vector<QMatrix4x4>& transformMatrices,
                                    QMatrix4x4 parentPosition, QMatrix4x4 parentTransformation){
    QMatrix4x4 childPosition;
    QMatrix4x4 childTransformation;
    childPosition = parentPosition;
    childTransformation = parentTransformation;
    childPosition.translate(_offX, _offY, _offZ);
    childTransformation = parentTransformation;
//    if (_parent != NULL){
//        childTransformation.rotate(this->_parent->_curRz, 0, 0, 1);
//        childTransformation.rotate(this->_parent->_curRy, 0, 1, 0);
//        childTransformation.rotate(this->_parent->_curRx, 1, 0, 0);
//    }
    childTransformation.translate(_offX, _offY, _offZ);
    childTransformation.translate(_curTx, _curTy, _curTz);

    childTransformation.rotate(_curRz, 0, 0, 1);
    childTransformation.rotate(_curRy, 0, 1, 0);
    childTransformation.rotate(_curRx, 1, 0, 0);

    bindedMatrices.push_back(childPosition);
    transformMatrices.push_back(childTransformation);
    for (int i = 0; i < _children.size(); i++){
        _children[i]->getChildTransformationMatrices(bindedMatrices, transformMatrices, childPosition, childTransformation);
    }
}


vector<trimesh::point> Joint::exportPositions(){
	if (_parent != NULL){
		std::cerr << "You can only call this method on root" << endl;
		return vector<trimesh::point>();
	}
	vector<trimesh::point> positions;
	QMatrix4x4 matrix;
	matrix.translate(_offX, _offY, _offZ); // global offset
//	matrix.translate(_curTx, _curTy, _curTz); // frame translation
//	matrix.rotate(_curRx, 1, 0, 0);
//	matrix.rotate(_curRy, 0, 1, 0);
//	matrix.rotate(_curRz, 0, 0, 1); // frame rotation
	QVector3D positionRoot;
	positionRoot = matrix * QVector3D(0, 0, 0);
	float x = float(positionRoot.x());
	float y = float(positionRoot.y());
	float z = float(positionRoot.z());
	trimesh::point currentPosition(x, y, z, 1);
	positions.push_back(currentPosition);
	exportChildPositions(matrix, positionRoot, positions);
	return positions;
}

void Joint::exportChildPositions(QMatrix4x4& matriceTransformation, QVector3D& positionRoot, vector<trimesh::point> &positions){
	for (int i = 0; i < this->_children.size(); i++){

		QMatrix4x4 matrix;
		matrix = matriceTransformation;
		matrix.translate(_offX, _offY, _offZ); // global offset
		QVector3D positionChild;
		positionChild = matriceTransformation * QVector3D(0, 0, 0);
		float x = float(positionChild.x());
		float y = float(positionChild.y());
		float z = float(positionChild.z());
		trimesh::point currentPosition(x, y, z, 1);
		positions.push_back(currentPosition);
		_children[i]->exportChildPositions(matriceTransformation, positionRoot, positions);
	}
	matriceTransformation.rotate(-_curRz, 0, 0, 1);
	matriceTransformation.rotate(-_curRy, 0, 1, 0);
	matriceTransformation.rotate(-_curRx, 1, 0, 0);
	matriceTransformation.translate(-_curTx, -_curTy, -_curTz);
	matriceTransformation.translate(-_offX, -_offY, -_offZ);
}

void Joint::exportPositions(QMatrix4x4& transform, vector<trimesh::point>& positions)
{
	transform.translate(_offX, _offY, _offZ);
	transform.translate(_curTx, _curTy, _curTz);
	transform.rotate(_curRz, 0, 0, 1);
	transform.rotate(_curRy, 0, 1, 0);
	transform.rotate(_curRx, 1, 0, 0);
	QVector3D pos = transform * QVector3D(0, 0, 0);
	float x = float(pos.x());
	float y = float(pos.y());
	float z = float(pos.z());
	trimesh::point vertex(x, y, z, 1.0);
	positions.push_back(vertex);
	for (int i=0; i<this->_children.size(); i++) {
		this->_children[i]->exportPositions(transform, positions);
	}
	transform.rotate(-_curRx, 1, 0, 0);
	transform.rotate(-_curRy, 0, 1, 0);
	transform.rotate(-_curRz, 0, 0, 1);
	transform.translate(-_curTx, -_curTy, -_curTz);
	transform.translate(-_offX, -_offY, -_offZ);
}

vector<trimesh::point> Joint::exportMiddleArticulations(){
    if (_parent != NULL){
        std::cerr << "You can only call this method on root" << endl;
        return vector<trimesh::point>();
    }
    vector<trimesh::point> positions;
    QMatrix4x4 matrix;
    QVector3D positionRoot;
    exportChildMiddleArticulations(matrix, positionRoot, positions);
    return positions;
}

void Joint::exportChildMiddleArticulations(QMatrix4x4& matriceTransformation, QVector3D& positionRoot, vector<trimesh::point> &positions){
    QMatrix4x4 matrix;
    matrix = matriceTransformation;
    matrix.translate(_offX, _offY, _offZ); // global offset
    QVector3D positionChild;
    positionChild = matrix * QVector3D(0, 0, 0);
    float x = float(positionChild.x());
    float y = float(positionChild.y());
    float z = float(positionChild.z());
    trimesh::point currentPosition(x, y, z, 1);

    if (this->_children.size() > 1){
        positions.push_back(currentPosition);
    } else if (this->_children.size() == 1){
        Joint *child = _children[0];
        QMatrix4x4 matrixChild;
        matrixChild = matrix;
        matrixChild.translate(child->_offX, child->_offY, child->_offZ); // global offset
        QVector3D positionChild;
        positionChild = matrixChild * QVector3D(0, 0, 0);
        float childX = float(positionChild.x());
        float childY = float(positionChild.y());
        float childZ = float(positionChild.z());
        trimesh::point currentPosition((x + childX)/2, (y + childY)/2, (z + childZ)/2, 1);
        positions.push_back(currentPosition);
    } else {
        positions.push_back(trimesh::point(100000, 100000, 100000, 1));
    }

    for (int i = 0; i < this->_children.size(); i++){
        _children[i]->exportChildMiddleArticulations(matrix, positionRoot, positions);
    }
}

Joint::~Joint() {
	for (int i = 0; i < _children.size(); i++){
		delete _children[i];
	}
	_dofs.clear();
	_children.clear();
}
