import os

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

def off_to_obj(off_filename, obj_filename):
    with open(off_filename, 'r') as file:
        lines = file.readlines()
    
    # Determine start of vertex and face data
    vertices = []
    faces = []
    vertex_count = int(lines[1].split()[0])  # Number of vertices
    face_count = int(lines[1].split()[1])    # Number of faces

    # Collect vertex data
    for line in lines[2:2 + vertex_count]:
        vertices.append(line.strip())

    # Collect face data
    for line in lines[2 + vertex_count:2 + vertex_count + face_count]:
        parts = line.split()
        if len(parts) > 1:
            # Shift indices by 1 for OBJ format (1-based indexing)
            faces.append(' '.join([str(int(index) + 1) for index in parts[1:]]))

    # Write to the OBJ file
    with open(obj_filename, 'w') as file:
        file.write("# OBJ file generated from OFF file\n")
        for vertex in vertices:
            file.write(f"v {vertex}\n")
        for face in faces:
            file.write(f"f {face}\n")

# Example usage
shape_names = ['beast', 'blade', 'boy', 'filigree', 'flower', 'hand_bones', 'octopus']

for shape_name in shape_names:
    input_file_path = os.path.join(shape_name, shape_name + '.off')  
    output_file_path = os.path.join(shape_name, shape_name + '.obj')  
    off_to_obj(input_file_path, output_file_path)

    input_file_path = os.path.join(shape_name, shape_name + '_bounding-proxy.off')  
    output_file_path = os.path.join(shape_name, shape_name + '_bounding-proxy.obj')  
    off_to_obj(input_file_path, output_file_path)
