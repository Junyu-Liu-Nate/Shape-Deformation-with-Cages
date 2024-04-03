#include "arap.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    // If this doesn't work for you, remember to change your working directory
    if (MeshLoader::loadTriMesh("meshes/peter.obj", vertices, triangles)) {
        m_shape.init(vertices, triangles);
    }

    heMesh.buildHalfEdgeStructure(vertices, triangles);

    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

// Move an anchored vertex, defined by its index, to targetPosition
void ARAP::move(int vertex, Vector3f targetPosition)
{
    std::vector<Eigen::Vector3f> new_vertices = m_shape.getVertices();
    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    // TODO: implement ARAP here
    iterativeOptimize(new_vertices, anchors, vertex, targetPosition);
//    iterativeOptimizeParallel(new_vertices, anchors, vertex, targetPosition);
    heMesh.updateVertexPos(new_vertices);

    // Here are some helpful controls for the application
    //
    // - You start in first-person camera mode
    //   - WASD to move, left-click and drag to rotate
    //   - R and F to move vertically up and down
    //
    // - C to change to orbit camera mode
    //
    // - Right-click (and, optionally, drag) to anchor/unanchor points
    //   - Left-click an anchored point to move it around
    //
    // - Minus and equal keys (click repeatedly) to change the size of the vertices

    m_shape.setVertices(new_vertices);
}

//----------------------- ARAP computation  -----------------------//
bool isPSD(const Eigen::MatrixXf& matrix) {
    Eigen::LDLT<Eigen::MatrixXf> ldlt(matrix);
    // A matrix is PSD if the diagonal elements of D in the LDLT decomposition are non-negative
    return (ldlt.info() == Eigen::Success) && (ldlt.vectorD().array() >= 0).all();
}

bool isPSDDetail(const Eigen::MatrixXf& matrix) {
    // Check if the matrix is symmetric
    if (matrix.transpose() != matrix) {
        std::cout << "L is not symmetric" << std::endl;
        std::cout << matrix << std::endl;
        return false;
    }

    // Compute the eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(matrix);
    if (eigenSolver.info() != Eigen::Success) {
        std::cout << "Eigenvalue computation did not converge" << std::endl;
        return false;
    }

    // Check if all eigenvalues are non-negative
    Eigen::VectorXf eigenvalues = eigenSolver.eigenvalues();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < -Eigen::NumTraits<float>::dummy_precision()) {
            // Considered negative if below the precision threshold
            std::cout << "Some eigenvalues are negative" << std::endl;
            return false;
        }
    }

    return true;
}

void ARAP::iterativeOptimize(std::vector<Eigen::Vector3f> new_vertices, const std::unordered_set<int>& anchors, int vertex, Vector3f targetPosition) {
    initialize(new_vertices, vertex, targetPosition);
    computeEdgeWeights();
    assembleL(anchors);

    Eigen::SparseMatrix<float> L_sparse = L.sparseView();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    solver.compute(L_sparse);

    if(solver.info() != Eigen::Success) {
        // Handle the error, the factorization failed
        std::cerr << "Decomposition failed." << std::endl;
        return;
    }

    int numIter = 10;
    for (int i = 0; i < numIter; i++) {
        computeVertexRotation();
        assenbleRHS(anchors);

        Eigen::MatrixXf p = solver.solve(RHS);
//        Eigen::MatrixXf p;
//        p.resize(new_vertices.size(), 3);
//        #pragma omp parallel for
//        for (int i = 0; i < 3; i++) {
//            p.col(i) = solver.solve(RHS.col(i));
//        }

        update(p, anchors);
    }
}

void ARAP::iterativeOptimizeParallel(std::vector<Eigen::Vector3f> new_vertices, const std::unordered_set<int>& anchors, int vertex, Vector3f targetPosition) {
    initialize(new_vertices, vertex, targetPosition);
    computeEdgeWeights();
    assembleL(anchors);

    Eigen::SparseMatrix<float> L_sparse = L.sparseView();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    solver.compute(L_sparse);

    if(solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed." << std::endl;
        return;
    }

    Eigen::MatrixXf p(new_vertices.size(), 3);

    int numIter = 10;
    for (int iter = 0; iter < numIter; ++iter) {
        computeVertexRotation();
        assenbleRHS(anchors);

        // Parallelizing the solver operation across the three columns of RHS
        QVector<int> cols = {0, 1, 2};
        QFutureSynchronizer<void> synchronizer;

        for (int col : cols) {
            synchronizer.addFuture(QtConcurrent::run([&solver, &p, this, col]() {
                p.col(col) = solver.solve(this->RHS.col(col));
            }));
        }

        synchronizer.waitForFinished();

        update(p, anchors);
    }
}

//---- Initialize vertices nextPositions (p') - previous position p (should be OK using new_vertices?)
void ARAP::initialize(std::vector<Eigen::Vector3f> new_vertices, int vertex, Vector3f targetPosition) {
#pragma omp parallel for
    for (int i = 0; i < new_vertices.size(); i++) {
        if (i == vertex) {
            heMesh.vertices.at(i).nextPosition = targetPosition;
        }
        else {
            heMesh.vertices.at(i).nextPosition = new_vertices.at(i);
        }
    }
}

//---- Update non-anchor vertices nextPositions (p')
void ARAP::update(Eigen::MatrixXf p, const std::unordered_set<int>& anchors) {
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        if (anchors.find(i) == anchors.end()) {
            heMesh.vertices.at(i).nextPosition = p.row(i);
        }
    }
}

//---- Compute edge weights
void ARAP::computeEdgeWeights() {
    // A map to ensure that twins have the same weight
    std::unordered_map<HalfEdge*, float> weights;

    for (HalfEdge& he : heMesh.halfEdges) {
        // Check if the weight has already been set by its twin
        if (weights.find(he.twin) != weights.end()) {
            he.weight = weights[he.twin]; // Use the same weight as the twin
        } else {
            // Calculate cotangent weight
            Vector3f alphaU = he.next->vertex->position - he.next->next->vertex->position;
            Vector3f alphaV = he.vertex->position - he.next->next->vertex->position;
            float alpha = cotangentOfAngle(alphaU, alphaV);

            Vector3f betaU = he.twin->next->vertex->position - he.twin->next->next->vertex->position;
            Vector3f betaV = he.twin->vertex->position - he.twin->next->next->vertex->position;
            float beta = cotangentOfAngle(betaU, betaV);

            float weight = 0.5 * (alpha + beta);

            he.weight = weight;
            weights[&he] = weight;
        }
    }
}

float ARAP::cotangentOfAngle(const Eigen::Vector3f& u, const Eigen::Vector3f& v) {
    float dot = u.dot(v);
    float crossMagnitude = u.cross(v).norm();

    // To prevent division by zero, check if crossMagnitude is near zero
    if (crossMagnitude < 1e-8) {
        // Handle the case where the angle is near 0 or 180 degrees
        // A cotangent of an angle close to 0 is theoretically infinite
        // while for an angle close to 180 degrees, it is near zero
        return std::numeric_limits<float>::infinity();  // or some other handling
    }

    return abs(dot / crossMagnitude);
}

//---- Compute per-vertex rotation matrix (R)
void ARAP::computeVertexRotation() {
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        Vertex* selectedVertex = &heMesh.vertices.at(i);

        int degree = heMesh.vertexDegree(selectedVertex);

        Eigen::MatrixXf P;
        P.resize(3, degree);
        Eigen::MatrixXf P_prime;
        P_prime.resize(3, degree);
        Eigen::MatrixXf D;
        D.resize(degree, degree);
        D.setIdentity();

        HalfEdge* iterateHalfEdge = selectedVertex->halfEdge;
        for (int j = 0; j < degree; j++) {
            P.col(j) = iterateHalfEdge->vertex->position - iterateHalfEdge->twin->vertex->position;
            P_prime.col(j) = iterateHalfEdge->vertex->nextPosition - iterateHalfEdge->twin->vertex->nextPosition;
            D(j, j) = iterateHalfEdge->weight;
            iterateHalfEdge = iterateHalfEdge->twin->next;
        }

        Matrix3f S = P * D * P_prime.transpose();

        // Compute the SVD
        JacobiSVD<Matrix3f> svd(S, ComputeFullU | ComputeFullV);
        Matrix3f Ui = svd.matrixU();
        Matrix3f Vi = svd.matrixV();

        // Compute R
        Eigen::Matrix3f R = Vi * Ui.transpose();

        // Ensuring that the determinant of Ri is positive
        if (R.determinant() < 0) {
            Ui.col(2) *= -1;
            R = Vi * Ui.transpose(); // Recompute R with the updated Ui
        }

        selectedVertex->R = R;
    }
}

//---- Assemble L and clear row/col for anchors
void ARAP::assembleL(const std::unordered_set<int>& anchors) {
    L.resize(heMesh.vertices.size(), heMesh.vertices.size());
    L.setZero();

    // Assemble L using weights
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        float totalWeight = 0.0f;

        HalfEdge* startEdge = heMesh.vertices.at(i).halfEdge;
        HalfEdge* edge = heMesh.vertices.at(i).halfEdge;
        do {
            totalWeight += edge->weight;
            int j = edge->twin->vertex->vertexIdx;
            L(i, j) = -edge->weight;
            edge = edge->twin->next;
        } while (edge && edge != startEdge);

        L(i,i) = totalWeight;
    }

    // Clear rows and cols for anchor points
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        if (anchors.find(i) != anchors.end()) {
            L.row(i).setZero();
            L.col(i).setZero();
            L(i,i) = 1.0f;
        }
    }
}

//---- Assemble RHS and clear row for anchors
void ARAP::assenbleRHS(const std::unordered_set<int>& anchors) {
    RHS.resize(heMesh.vertices.size(), 3);

    // Assemble RHS using
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        Vector3f totalB = Vector3f(0, 0 ,0);

        HalfEdge* startEdge = heMesh.vertices.at(i).halfEdge;
        HalfEdge* edge = startEdge;
        do {
            float weight = edge->weight;
            Matrix3f RSum = edge->vertex->R + edge->twin->vertex->R;
            Vector3f pDiff = edge->vertex->position - edge->twin->vertex->position;
            totalB += 0.5 * weight * RSum * pDiff;
            edge = edge->twin->next;
        } while (edge && edge != startEdge);

        RHS.row(i) = totalB;
    }

    // Update the "deleted" rows (when j != i)
#pragma omp parallel for
    for (int j = 0; j < heMesh.vertices.size(); j++) {
        if (anchors.find(j) != anchors.end()) {
            for (int i = 0; i < heMesh.vertices.size(); i++) {
                float weight_ij = 0.0f;
                HalfEdge* startEdge = heMesh.vertices.at(i).halfEdge;
                HalfEdge* edge = startEdge;
                do {
                    if (edge->twin->vertex->vertexIdx == j) {
                        weight_ij = edge->weight;
                        break;
                    }
                    edge = edge->twin->next;
                } while (edge && edge != startEdge);

                RHS.row(i) += weight_ij * heMesh.vertices.at(j).nextPosition;
            }
        }
    }

    // Update the "deleted" rows (diagonal position when j == i)
#pragma omp parallel for
    for (int i = 0; i < heMesh.vertices.size(); i++) {
        if (anchors.find(i) != anchors.end()) {
            RHS.row(i) = heMesh.vertices.at(i).nextPosition;
        }
    }
}
