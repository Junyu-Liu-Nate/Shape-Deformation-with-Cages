def convert_obj_to_simple_format(input_file_path, output_file_path):
    vertices = []  # List to store vertex positions
    faces = []  # List to store faces

    # Read the input OBJ file
    with open(input_file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if not parts:
                continue  # Skip empty lines

            prefix = parts[0]
            if prefix == 'v':  # Vertex position
                # Add vertex position to list
                vertices.append(' '.join(parts[1:]))
            elif prefix == 'f':  # Face definition
                # Extract just the vertex indices for each face, ignore texture and normal indices
                face_vertex_indices = ['/'.join(vertex.split('/')[:1]) for vertex in parts[1:]]
                faces.append(' '.join(face_vertex_indices))

    # Write the output OBJ file
    with open(output_file_path, 'w') as file:
        # Write vertex positions
        for vertex in vertices:
            file.write(f'v {vertex}\n')
        file.write('\n')  # Add a blank line before faces for readability
        # Write faces
        for face in faces:
            file.write(f'f {face}\n')

# Example usage
input_file_path = 'cat_cage.obj'  # Replace with your input file path
output_file_path = 'cat_cage_converted.obj'  # Replace with your desired output file path
convert_obj_to_simple_format(input_file_path, output_file_path)
