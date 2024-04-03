#version 330 core
out vec4 fragColor;

in vec3 normal_cameraSpace;

uniform int   wire  = 0;
uniform float red   = 1.0;
uniform float green = 1.0;
uniform float blue  = 1.0;
uniform float alpha = 1.0;

void main() {
    // Do lighting in camera space
    vec3 lightDir = normalize(vec3(0, 0.5, 1));
    float c = clamp(dot(normal_cameraSpace, lightDir), 0, 1);

    fragColor = vec4(red * c, green * c, blue * c, 1);
}
