#version 330 core

layout (location = 0) in vec3 position;
layout (location = 3) in vec2 inUvPos;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

out vec2 uvPos;

void main() {
    uvPos = inUvPos;
    // gl_Position = vec4(position, 1.0);
    gl_Position = proj * view * model * vec4(position, 1.0);
    // gl_Position = view * vec4(position, 1.0);
}
