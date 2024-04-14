#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 inUvPos;

out vec2 uvPos;

void main() {
    uvPos = inUvPos;
    gl_Position = vec4(position, 1.0);
}
