#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

// declare an interface block; see 'Advanced GLSL' for what these are.
out vec3 FragPos;
out vec3 Normal;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main()
{
    FragPos = aPos;
    Normal = aNormal;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
