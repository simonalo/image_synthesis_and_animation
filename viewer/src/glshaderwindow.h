#ifndef GLSHADERWINDOW_H
#define GLSHADERWINDOW_H

#include "openglwindow.h"
#include "TriMesh.h"
#include "joint.h"
#include "weight.h"

#include <QtGui/QGuiApplication>
#include <QtGui/QMatrix4x4>
#include <QtGui/QOpenGLShaderProgram>
#include <QOpenGLFramebufferObject>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QtGui/QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QScreen>
#include <QMouseEvent>
#include <QLabel>

class glShaderWindow : public OpenGLWindow
{
    Q_OBJECT
public:
    glShaderWindow(QWindow *parent = 0);
    ~glShaderWindow();

    void initialize();
    void render();
    void resize(int x, int y);
    void setWorkingDirectory(QString& myPath, QString& myName, QString& texture, QString& envMap, QString& mySkeleton, QString& weightsName);
    inline const QString& getWorkingDirectory() { return workingDirectory;};
    inline const QStringList& fragShaderSuffix() { return m_fragShaderSuffix;};
    inline const QStringList& vertShaderSuffix() { return m_vertShaderSuffix;};
    void calculateNewPosition(vector<QMatrix4x4>& transformMatrices, vector<QMatrix4x4>& offsetMatrix);
    // Override of parent
    void renderNow(){
        if (getAnimating()){

            struct timespec start;
            clock_gettime(CLOCK_MONOTONIC, &start);
            double elapsed = start.tv_sec +  - timeLastFrame + (start.tv_nsec) / 1000000000.0;

            if ((int)(elapsed / Joint::FRAME_TIME) <= FRAME){
                // cerr << "WAITING" << endl;

                renderLater();
                return;
            } else if ((int)(elapsed / Joint::FRAME_TIME) > FRAME){
                frame = (int)(elapsed / Joint::FRAME_TIME);
            }
        }

        OpenGLWindow::renderNow();

    }
    void toggleAnimating(){
        OpenGLWindow::toggleAnimating();
        if (getAnimating()){
            struct timespec start;
            clock_gettime(CLOCK_MONOTONIC, &start);
            timeLastFrame = start.tv_sec + start.tv_nsec/1000000000.0 - FRAME * Joint::FRAME_TIME;
        }
    }

public slots:
    void openSceneFromFile();
    void openNewTexture();
    void openSkeletonFromFile();
    void openWeightsForSkeleton();
    void openNewEnvMap();
    void saveScene();
    void toggleFullScreen();
    void saveScreenshot();
    QWidget* makeAuxWindow();
    void setWindowSize(const QString& size);
    void setShader(const QString& size);
    void cookTorranceClicked();
    void blinnPhongClicked();
    void transparentClicked();
    void opaqueClicked();
    void updateLightIntensity(int lightSliderValue);
    void updateShininess(int shininessSliderValue);
    void updateEta(int etaSliderValue);
    void changeFrame(int frame);

protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void timerEvent(QTimerEvent *e);
    void resizeEvent(QResizeEvent * ev);
    void wheelEvent(QWheelEvent * ev);
    void keyPressEvent(QKeyEvent* e);


private:
    clock_t tStart;

    QOpenGLShaderProgram* prepareShaderProgram(const QString& vertexShaderPath, const QString& fragmentShaderPath);
    QOpenGLShaderProgram* prepareComputeProgram(const QString& computeShaderPath);
    void createSSBO();
    void bindSceneToProgram();
    void initializeTransformForScene();
    void initPermTexture();
    void loadTexturesForShaders();
    void openScene();
    void openSkeleton();
    void openWeights();
    void mouseToTrackball(QVector2D &in, QVector3D &out);
    void fillValuesFromJoints(Joint* current, const vector<trimesh::point>& _vert);
    QLabel* frameLabelValue; 
    
    int FRAME=0;

    vector<trimesh::point> initVertices;

    // Are we using GPGPU?
    bool isGPGPU;
    bool isFullRt;
    // Are we using compute shaders?
    bool hasComputeShaders;
    // Model we are displaying:
    QString  workingDirectory;
    QString  modelName;
    QString  skeletonName;
    QString  weightsName;
    QString  textureName;
    QString  envMapName;
    trimesh::TriMesh* modelMesh;
    Joint* skeleton; // Add skeleton
    uchar* pixels;
    // Ground
    trimesh::point *g_vertices;
    trimesh::vec *g_normals;
    trimesh::vec2 *g_texcoords;
    trimesh::point *g_colors;
    int *g_indices;
    int g_numPoints;
    int g_numIndices;
    // Skeleton (Everything is made like ground)
    trimesh::point *s_vertices;
    trimesh::point *s_colors;
    trimesh::vec2 *s_texcoords;
    trimesh::vec *s_normals;
    int frame;

    int *s_indices;
    int s_numPoints;
    int s_numIndices;
    // Weights
    vector<Weight> VerticesWeights;
    // GPGPU
    trimesh::point *gpgpu_vertices;
    trimesh::vec *gpgpu_normals;
    trimesh::vec2 *gpgpu_texcoords;
    trimesh::point *gpgpu_colors;
    int *gpgpu_indices;
    int compute_groupsize_x;
    int compute_groupsize_y;
    // ComputeShader:
    GLuint ssbo[4];
    // Parameters controlled by UI
    bool blinnPhong;
    bool transparent;
    float eta;
    float lightIntensity;
    float shininess;
    float lightDistance;
    float groundDistance;
    int maxBounds;

    QString shaderName;


    // OpenGL variables encapsulated by Qt
    QOpenGLShaderProgram *m_program;
    QOpenGLShaderProgram *ground_program;
    QOpenGLShaderProgram *skeleton_program;
    QOpenGLShaderProgram *compute_program;
    QOpenGLShaderProgram *shadowMapGenerationProgram;
    QOpenGLTexture* environmentMap;
    QOpenGLTexture* texture;
    QOpenGLTexture* permTexture;   // for Perlin noise
    QOpenGLTexture* computeResult; // output of compute shader
    // Model
    QOpenGLBuffer m_vertexBuffer;
    QOpenGLBuffer m_indexBuffer;
    QOpenGLBuffer m_normalBuffer;
    QOpenGLBuffer m_colorBuffer;
    QOpenGLBuffer m_texcoordBuffer;
    QOpenGLVertexArrayObject m_vao;
    int m_numFaces;
	QVector3D m_center;
	QVector3D m_bbmin;
	QVector3D m_bbmax;
    // Ground
    QOpenGLVertexArrayObject ground_vao;
    QOpenGLBuffer ground_vertexBuffer;
    QOpenGLBuffer ground_indexBuffer;
    QOpenGLBuffer ground_normalBuffer;
    QOpenGLBuffer ground_colorBuffer;
    QOpenGLBuffer ground_texcoordBuffer;
    // skeleton
    QOpenGLVertexArrayObject skeleton_vao;
    QOpenGLBuffer skeleton_vertexBuffer;
    QOpenGLBuffer skeleton_indexBuffer;
    QOpenGLBuffer skeleton_colorBuffer;
    QOpenGLBuffer skeleton_texcoordBuffer;
    QOpenGLBuffer skeleton_normalBuffer;
    // Matrix for all objects
    QMatrix4x4 m_matrix[3]; // 0 = object, 1 = light, 2 = ground
    QMatrix4x4 m_perspective;
    // Shadow mapping
    GLuint shadowMap_fboId;
    GLuint shadowMap_rboId;
    GLuint shadowMap_textureId;
    int shadowMapDimension;
    // User interface variables
    bool fullScreenSnapshots;
    QStringList m_fragShaderSuffix;
    QStringList m_vertShaderSuffix;
    QStringList m_compShaderSuffix;
    QVector2D lastMousePosition;
    QVector3D lastTBPosition;
    Qt::MouseButton mouseButton;
    float m_screenSize; // max window dimension
    QWidget* auxWidget; // window for parameters
    QWidget* container;
};

#endif // GLSHADERWINDOW_H
