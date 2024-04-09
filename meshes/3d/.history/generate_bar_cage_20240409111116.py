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
    layer_size = (divisions + 1)**2  # Number of points per layer

    # Generate faces for each cube in the grid
    for layer in range(divisions):
        for row in range(divisions):
            for col in range(divisions):
                # Calculate the index in the 1D list of the current corner of the cube
                index = layer * layer_size + row * (divisions + 1) + col

                # Get the indices of the 8 corners of the current cube
                p0 = index
                p1 = index + 1
                p2 = index + divisions + 1
                p3 = index + divisions + 2
                p4 = index + layer_size
                p5 = index + layer_size + 1
                p6 = index + layer_size + divisions + 1
                p7 = index + layer_size + divisions + 2

                # Create two triangles for each face of the cube
                # Bottom face
                faces.append((p0, p2, p1))
                faces.append((p1, p2, p3))
                # Top face
                faces.append((p4, p5, p6))
                faces.append((p5, p7, p6))
                # Front face
                faces.append((p0, p1, p4))
                faces.append((p1, p5, p4))
                # Back face
                faces.append((p2, p6, p3))
                faces.append((p3, p6, p7))
                # Left face
                faces.append((p0, p4, p2))
                faces.append((p2, p4, p6))
                # Right face
                faces.append((p1, p3, p5))
                faces.append((p3, p7, p5))

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
