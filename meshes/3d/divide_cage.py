def divide_long_edges(obj_file_path):
    with open(obj_file_path, 'r') as file:
        lines = file.readlines()

    vertices = []
    faces = []

    # Parse vertices and faces from the OBJ file
    for line in lines:
        if line.startswith('v '):
            vertex = list(map(float, line.strip().split()[1:]))
            vertices.append(vertex)
        elif line.startswith('f '):
            face = list(map(int, line.strip().split()[1:]))
            faces.append(face)

    new_vertices = []

    # Divide long edges into three parts
    for face in faces:
        v1 = vertices[face[0] - 1]
        v2 = vertices[face[1] - 1]
        v3 = vertices[face[2] - 1]

        edge_lengths = [
            ((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2 + (v1[2] - v2[2]) ** 2) ** 0.5,  # Edge 1-2
            ((v2[0] - v3[0]) ** 2 + (v2[1] - v3[1]) ** 2 + (v2[2] - v3[2]) ** 2) ** 0.5,  # Edge 2-3
            ((v3[0] - v1[0]) ** 2 + (v3[1] - v1[1]) ** 2 + (v3[2] - v1[2]) ** 2) ** 0.5   # Edge 3-1
        ]

        # Find the index of the longest edge
        longest_edge_index = edge_lengths.index(max(edge_lengths))

        # Divide the longest edge into three segments
        if longest_edge_index == 0:
            midpoint = [(v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2, (v1[2] + v2[2]) / 2]
            new_vertices.append(midpoint)
        elif longest_edge_index == 1:
            midpoint = [(v2[0] + v3[0]) / 2, (v2[1] + v3[1]) / 2, (v2[2] + v3[2]) / 2]
            new_vertices.append(midpoint)
        else:
            midpoint = [(v3[0] + v1[0]) / 2, (v3[1] + v1[1]) / 2, (v3[2] + v1[2]) / 2]
            new_vertices.append(midpoint)

    # Update face definitions to include new vertices
    new_faces = []
    for i, face in enumerate(faces):
        v1, v2, v3 = face
        v4 = len(vertices) + i + 1
        v5 = len(vertices) + len(faces) + i + 1

        new_faces.extend([
            [v1, v4, v5],
            [v4, v2, v5],
            [v5, v2, v3]
        ])

    # Write the updated mesh to a new OBJ file
    output_file_path = obj_file_path.replace('.obj', '_divided.obj')
    with open(output_file_path, 'w') as file:
        for vertex in vertices:
            file.write(f"v {' '.join(map(str, vertex))}\n")
        for vertex in new_vertices:
            file.write(f"v {' '.join(map(str, vertex))}\n")
        for face in new_faces:
            file.write(f"f {' '.join(map(str, face))}\n")

    return output_file_path

# Example usage
original_obj_file_path = "simple/cactus/cactus_cube_cage.obj"
divided_obj_file_path = divide_long_edges(original_obj_file_path)
print(f"Mesh with divided long edges saved to: {divided_obj_file_path}")
