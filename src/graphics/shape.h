#pragma once

#include <GL/glew.h>
#include <vector>
#include <unordered_set>

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include "Eigen/StdVector"
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)
#include "Eigen/Dense"

enum SelectMode
{
    None     = 0,
    Anchor   = 1,
    Unanchor = 2
};

class Shader;

class Shape
{
public:
    Shape();

    void init(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3i> &triangles);
    void initWithTexture(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector3i> &triangles,
                         const std::vector<Eigen::Vector2f> &uvCoords,
                         const std::string &textureFilePath);
    void initPoints(const std::vector<Eigen::Vector3f> &vertices);
    void setVertices(const std::vector<Eigen::Vector3f> &vertices);
    void setVertices2d(const std::vector<Eigen::Vector3f> &vertices);
    void setCtrlPtsVertices(int vertex, const Eigen::Vector3f &targetPosition);

    void setModelMatrix(const Eigen::Affine3f &model);

    void draw(Shader *shader, GLenum mode);
    SelectMode select(Shader *shader, int vertex);
    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode);
    int  getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold);
    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start);

    const std::vector<Eigen::Vector3f>& getVertices();
    const std::vector<Eigen::Vector3i>& getFaces();
    const std::unordered_set<int>& getAnchors();

    void clearAnchors() {
        m_anchors.clear();

        if (!m_isCtrlPtShape) {
            selectHelper();
        } else {
            selectHelperForCtrlPt();
        }
    }

private:
    GLuint m_surfaceVao;
    GLuint m_surfaceVbo;
    GLuint m_surfaceIbo;
    GLuint m_surfaceTexture;

    unsigned int m_numSurfaceVertices;
    unsigned int m_verticesSize;
    float m_red;
    float m_blue;
    float m_green;
    float m_alpha;
    bool m_isTextured = false;
    bool m_isCtrlPtShape = false;

    std::vector<Eigen::Vector3i> m_faces;
    std::vector<Eigen::Vector3f> m_vertices;
    std::unordered_set<int>      m_anchors;
    std::vector<Eigen::Vector2f> m_uvs;

    Eigen::Matrix4f m_modelMatrix;
    int lastSelected = -1;

    // Helpers

    void selectHelper();
    void selectHelperForCtrlPt();
    Eigen::Vector3f getNormal(const Eigen::Vector3i& face);
    void updateMesh(const std::vector<Eigen::Vector3i> &triangles,
                    const std::vector<Eigen::Vector3f> &vertices,
                           std::vector<Eigen::Vector3f>& verts,
                           std::vector<Eigen::Vector3f>& normals,
                           std::vector<Eigen::Vector3f>& colors);
    void updateTexturedMesh(const std::vector<Eigen::Vector3i> &triangles,
                      const std::vector<Eigen::Vector3f> &vertices,
                      std::vector<Eigen::Vector3f>& verts,
                      std::vector<Eigen::Vector3f>& normals,
                      std::vector<Eigen::Vector3f>& colors,
                      const std::vector<Eigen::Vector2f>& uvCoords);
    void updateCtrlPtColor(std::vector<Eigen::Vector3f>& colors);
};
