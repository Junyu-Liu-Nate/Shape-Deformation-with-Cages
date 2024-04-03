#version 330 core

layout(points) in;
layout (triangle_strip, max_vertices = 4) out;

in vec4 vColor[];
out vec4  fColor;
out vec2  fPoint;
out float fRadius;

uniform int width;
uniform int height;
uniform float vSize;

void main() {
    vec4 offset = vec4(vSize, -vSize, 0.0, 0.0);

    int i;
    for (i = 0; i < gl_in.length(); i ++) {
        vec4 p = gl_in[i].gl_Position;

        fColor = vColor[i];
        fPoint = vec2((p.x * 0.5 + 0.5) * width,
                      (p.y * 0.5 + 0.5) * height);
        fRadius = vSize;

        gl_Position = p + offset.yxzw;
        EmitVertex();

        gl_Position = p + offset.yyzw;
        EmitVertex();

        gl_Position = p + offset.xxzw;
        EmitVertex();

        gl_Position = p + offset.xyzw;
        EmitVertex();

        EndPrimitive();
    }
}
