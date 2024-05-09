# Shape-Deformation-with-Green-Coordinates
**1. Project Overview:**


This is an interactive cage-based deformation program based on 4 papers. It allows the deformation of 2D and 3D shapes in Mean-Value-Coordinates and Green-Coordinates with both full and partial cage, and 2D deformation in high-order Green Coordinates in full cage.

|                               | 2D Full Cage       | 2D Partial Cage    | 3D Full Cage       | 3D Partial Cage    |
|-------------------------------|:------------------:|:------------------:|:------------------:|:------------------:|
| Green Coordinates             | Implemented [1]    | Implemented [1]    | Implemented [1]    | Implemented [1]    |
| Mean Value Coordinates        | Implemented [2]    | Implemented [2]    | Implemented [3]    | Implemented [3]    |
| Higher Order Green Coordinates| Implemented [4]    | -                  | -                  | -                  |

**2. Background:**

There exist two major categories of deformation methods: Directly modifying the shape geometry and shape-based deformation. The first kind, exemplifying with [ARAP](https://igl.ethz.ch/projects/ARAP/arap_web.pdf), mostly define a deformative energy and iteratively optimize it so that the energy is minimized. While it is considered as robust and simple, it also suffered the drawbacks of discretization and optimization errors, inconvenient control on large/dense meshes, and a relatively slow operation on extensive shapes.

Shape-based deformation, on the other hand, uses arbitrarily coarse proxy of shapes as cage to generate deformation. Modifying the cage vertices would correspondingly and smoothly deform the shape. It is both intuitive and easy to control and fast on dense meshes as well.

|            | Shape Geometry Deformation       | Cage Deformation    |
|------------|:--------------------------------:|:-------------------:|
| Method     | Define deformative energy and minimize it for shape preservation | Map all vertices on the mesh to an edge/face on the cage for deformation |
| Advantages | Robust and simple                  | Intuitive, Robust and Fast    |
| Drawbacks  | Discretization/optimization errors, inconvenient control, slow on dense meshes    | - |


In this program, we use a cage-based deformation system to demonstrate two kinds of barycentric coordinates: Mean-Value-Coordinates and Green Coordinates. MVC tend to enable strict affine deformation with cage movements, and Green Coordinate preserve shape in deformation. Their computation equation and derivation reference can be found in the following chart.

|                        | 2D MVC             | 2D Green           | 3D MVC             | 3D Green           |
|------------------------|:------------------:|:------------------:|:------------------:|:------------------:|
| Equation               | $\eta = F(\eta, P) = \sum \phi_i(\eta)\eta_i$    |$\eta = F(r, P) = \sum_{i \in I_v} (a_i m_i x_i) + \sum_{j \in I_f} (b_j h_j s_j(t_j))$    |  $\eta = F(\eta, P) = \sum \phi_i(\eta)\eta_i$    | $\eta=F(\eta ; P)=\sum_{i \in I_V} \sum_{n=0}^{N_c} \varphi_i^n(\eta) c_i^n+\sum_{j \in I_T} \sum_{n=0}^{N_c} \psi_j^n(\eta) c_j^{\perp}$ |
| Derivation Reference   | [2]    | [1] | [3]    | [1]    |

The derivation of Higher-order GC can be found in [literature [4]](https://dl.acm.org/doi/10.1145/3588432.3591499).



**4. References:**

References can be found here:

[[1] Lipman, Y., Levin, D., & Cohen-Or, D. (2008). Green coordinates. ACM transactions on graphics (TOG), 27(3), 1-10.](https://dl.acm.org/doi/10.1145/1360612.1360677#:~:text=The%20coordinates%20are%20motivated%20by,with%20a%20shape%2Dpreserving%20property.)

[2] Hormann, K., & Floater, M. S. (2006). Mean value coordinates for arbitrary planar polygons. ACM Transactions on Graphics (TOG), 25(4), 1424-1441.](https://dl.acm.org/doi/10.1145/1183287.1183295)

[[3] Ju, T., Schaefer, S., & Warren, J. (2023). Mean value coordinates for closed triangular meshes. In Seminal Graphics Papers: Pushing the Boundaries, Volume 2 (pp. 223-228).](https://www.cse.wustl.edu/~taoju/research/meanvalue.pdf)

[4] Michel, É., & Thiery, J. M. (2023, July). Polynomial 2D Green Coordinates for Polygonal Cages. In ACM SIGGRAPH 2023 Conference Proceedings (pp. 1-9).](https://dl.acm.org/doi/10.1145/3588432.3591499)
