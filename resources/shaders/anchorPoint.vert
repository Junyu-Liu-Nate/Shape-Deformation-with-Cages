#version 330 core

layout(location = 0) in vec3 position; // Position of the vertex
layout(location = 2) in vec3 vcolor;   // Normal of the vertex

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

out vec4 vColor;

void main() {
    vColor = vec4(vcolor, 1.0);

    gl_Position = proj * view * model * vec4(position, 1.0);
}
