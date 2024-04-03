#include "shape.h"

#include <iostream>
#include "graphics/shader.h"

using namespace Eigen;
using namespace std;

// ================== Constructor

Shape::Shape() :
    m_surfaceVao(),
    m_surfaceVbo(),
    m_surfaceIbo(),
    m_numSurfaceVertices(),
    m_verticesSize(),
    m_red(),
    m_blue(),
    m_green(),
    m_alpha(),
    m_faces(),
    m_vertices(),
    m_anchors(),
    m_modelMatrix(Matrix4f::Identity()),
    lastSelected(-1)
{}

// ================== Initialization and Updating

void Shape::init(const vector<Vector3f> &vertices, const vector<Vector3i> &triangles)
{
    m_vertices.clear();
    copy(vertices.begin(), vertices.end(), back_inserter(m_vertices));

    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;
    vector<Vector3i> faces;
    faces.reserve(triangles.size());

    for (int s = 0; s < triangles.size() * 3; s+=3) faces.push_back(Vector3i(s, s + 1, s + 2));
    updateMesh(triangles, vertices, verts, normals, colors);

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * faces.size(), static_cast<const void *>(faces.data()), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(sizeof(float) * verts.size() * 3));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(sizeof(float) * (verts.size() * 3 + colors.size() * 3)));

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_numSurfaceVertices = faces.size() * 3;
    m_verticesSize = vertices.size();
    m_faces = triangles;
    m_red   = 0.5f + 0.5f * rand() / ((float) RAND_MAX);
    m_blue  = 0.5f + 0.5f * rand() / ((float) RAND_MAX);
    m_green = 0.5f + 0.5f * rand() / ((float) RAND_MAX);
    m_alpha = 1.0f;
}

void Shape::setVertices(const vector<Vector3f> &vertices)
{
    m_vertices.clear();
    copy(vertices.begin(), vertices.end(), back_inserter(m_vertices));

    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;

    updateMesh(m_faces, vertices, verts, normals, colors);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ================== Model Matrix

void Shape::setModelMatrix(const Affine3f &model) { m_modelMatrix = model.matrix(); }

// ================== General Graphics Stuff

void Shape::draw(Shader *shader, GLenum mode)
{
    Eigen::Matrix3f m3 = m_modelMatrix.topLeftCorner(3, 3);
    Eigen::Matrix3f inverseTransposeModel = m3.inverse().transpose();

    switch(mode) {
    case GL_TRIANGLES:
    {
        shader->setUniform("wire", 0);
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        shader->setUniform("red",   m_red);
        shader->setUniform("green", m_green);
        shader->setUniform("blue",  m_blue);
        shader->setUniform("alpha", m_alpha);
        glBindVertexArray(m_surfaceVao);
        glDrawElements(mode, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        break;
    }
    case GL_POINTS:
    {
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        glBindVertexArray(m_surfaceVao);
        glDrawElements(mode, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        break;
    }
    }
}

SelectMode Shape::select(Shader *shader, int closest_vertex)
{
    if (closest_vertex == -1) return SelectMode::None;

    bool vertexIsNowSelected = m_anchors.find(closest_vertex) == m_anchors.end();

    if (vertexIsNowSelected) {
        m_anchors.insert(closest_vertex);
    } else {
        m_anchors.erase(closest_vertex);
    }

    selectHelper();

    return vertexIsNowSelected ? SelectMode::Anchor : SelectMode::Unanchor;
}

bool Shape::selectWithSpecifiedMode(Shader *shader, int closest_vertex, SelectMode mode)
{
    switch (mode) {
    case SelectMode::None: {
        return false;
    }
    case SelectMode::Anchor: {
        if (m_anchors.find(closest_vertex) != m_anchors.end()) return false;
        m_anchors.insert(closest_vertex);
        break;
    }
    case SelectMode::Unanchor: {
        if (m_anchors.find(closest_vertex) == m_anchors.end()) return false;
        m_anchors.erase(closest_vertex);
        break;
    }
    }

    selectHelper();

    return true;
}

int Shape::getClosestVertex(Vector3f start, Vector3f ray, float threshold)
{
    int closest_vertex = -1;
    int i = 0;
    float dist = numeric_limits<float>::max();
    ParametrizedLine line = ParametrizedLine<float, 3>::Through(start, start + ray);

    for (const Vector3f &v : m_vertices) {
        float d = line.distance(v);
        if (d<dist) {
            dist = d;
            closest_vertex = i;
        }
        ++i;
    }

    if (dist >= threshold) closest_vertex = -1;

    return closest_vertex;
}

bool Shape::getAnchorPos(int lastSelected,
                         Eigen::Vector3f& pos,
                         Eigen::Vector3f  ray,
                         Eigen::Vector3f  start)
{
    bool isAnchor = m_anchors.find(lastSelected) != m_anchors.end();
    if (isAnchor) {
        Eigen::Vector3f oldPos = m_vertices[lastSelected];
        Eigen::ParametrizedLine line = ParametrizedLine<float, 3>::Through(start, start+ray);
        pos = line.projection(oldPos);
    }
    return isAnchor;
}

// ================== Accessors

const vector<Vector3f>   &Shape::getVertices() { return m_vertices; }
const vector<Vector3i>   &Shape::getFaces()    { return m_faces;    }
const unordered_set<int> &Shape::getAnchors()  { return m_anchors;  }

// ================== Helpers

void Shape::selectHelper()
{
    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;
    updateMesh(m_faces, m_vertices, verts, normals, colors);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3) + (colors.size() * 3)), nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * verts.size() * 3, static_cast<const void *>(verts.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * verts.size() * 3, sizeof(float) * normals.size() * 3, static_cast<const void *>(normals.data()));
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(float) * ((verts.size() * 3) + (normals.size() * 3)), sizeof(float) * colors.size() * 3, static_cast<const void *>(colors.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Vector3f Shape::getNormal(const Vector3i& face)
{
    Vector3f& v1 = m_vertices[face[0]];
    Vector3f& v2 = m_vertices[face[1]];
    Vector3f& v3 = m_vertices[face[2]];
    Vector3f e1 = v2 - v1;
    Vector3f e2 = v3 - v1;
    Vector3f n = e1.cross(e2);
    return n.normalized();
}

void Shape::updateMesh(const std::vector<Eigen::Vector3i> &faces,
                       const std::vector<Eigen::Vector3f> &vertices,
                       std::vector<Eigen::Vector3f>& verts,
                       std::vector<Eigen::Vector3f>& normals,
                       std::vector<Eigen::Vector3f>& colors)
{
    verts.reserve(faces.size() * 3);
    normals.reserve(faces.size() * 3);

    for (const Eigen::Vector3i& face : faces) {
        Vector3f n = getNormal(face);

        for (auto& v: {face[0], face[1], face[2]}) {
            normals.push_back(n);
            verts.push_back(vertices[v]);

            if (m_anchors.find(v) == m_anchors.end()) {
                colors.push_back(Vector3f(1,0,0));
            } else {
                colors.push_back(Vector3f(0, 1 - m_green, 1 - m_blue));
            }
        }
    }
}

