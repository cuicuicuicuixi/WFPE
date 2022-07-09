#version 330 core

attribute highp vec3 aPosi;
attribute highp vec2 aTexCoords;

out vec2 TexCoords;

void main()
{
    TexCoords = aTexCoords;
    gl_Position = vec4(aPosi, 1.0);
}
