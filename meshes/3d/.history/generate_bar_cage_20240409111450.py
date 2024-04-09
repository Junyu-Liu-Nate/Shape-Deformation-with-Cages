import numpy as np

# Function to generate a grid of points for a cage
def generate_cage_points(size, divisions):
    points = []
    step = size / divisions
    for x in np.linspace(-size / 2, size / 2, divisions):
        for y in np.linspace(-size / 2, size / 2, divisions):
            for z in np.linspace(-size / 2, size / 2, divisions):
                points.append((x, y, z))
    return np.array(points)

# Function to create faces from the points
def create_faces(points, divisions):
    faces = []
    points_per_layer = (divisions + 1) ** 2
    points_per_row = divisions + 1

    # Function to calculate the linear index in the points array
    def idx(layer, row, col):
        return layer * points_per_layer + row * points_per_row + col

    # Generate faces for the subdivided volume
    for layer in range(divisions):
        for row in range(divisions):
            for col in range(divisions):
                # Vertices of the current cube
                p0 = idx(layer, row, col)
                p1 = idx(layer, row, col + 1)
                p2 = idx(layer, row + 1, col)
                p3 = idx(layer, row + 1, col + 1)
                p4 = idx(layer + 1, row, col)
                p5 = idx(layer + 1, row, col + 1)
                p6 = idx(layer + 1, row + 1, col)
                p7 = idx(layer + 1, row + 1, col + 1)

                # Add faces - for each cube, 2 triangles per face
                # Front face
                faces.append((p0, p2, p3))
                faces.append((p0, p3, p1))
                # Back face
                faces.append((p5, p7, p6))
                faces.append((p5, p6, p4))
                # Left face
                faces.append((p4, p6, p2))
                faces.append((p4, p2, p0))
                # Right face
                faces.append((p1, p3, p7))
                faces.append((p1, p7, p5))
                # Top face
                faces.append((p4, p0, p1))
                faces.append((p4, p1, p5))
                # Bottom face
                faces.append((p2, p6, p7))
                faces.append((p2, p7, p3))

    # Convert to 1-based indexing for OBJ format
    faces = [(f[0] + 1, f[1] + 1, f[2] + 1) for f in faces]

    return faces



# Generate points for a more complex bar cage
cage_size = 2  # Size of the cage
divisions = 4  # Number of divisions per axis
cage_points = generate_cage_points(cage_size, divisions)

# Create faces for the complex bar cage
cage_faces = create_faces(cage_points, divisions)

# Write to an OBJ file format
cage_obj = "# Complex Bar Cage Mesh\no Complex_Bar_Cage\n"
for v in cage_points:
    cage_obj += "v {} {} {}\n".format(*v)
for f in cage_faces:
    cage_obj += "f {}\n".format(' '.join(map(str, f)))

# Save to a file
obj_file_path = 'complex_bar_cage.obj'
with open(obj_file_path, "w") as file:
    file.write(cage_obj)
