#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform sampler2D floorTexture;
uniform vec3 camPos;

vec3 lightPos = vec3(10000, 10000, 10000);
void main()
{
    vec3 color = vec3(1,1,1);
    // ambient
    vec3 ambient = 0.05 * color;
    // diffuse
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 normal = normalize(Normal);
    float diff = max(dot(lightDir, normal), 0);
    vec3 diffuse = diff * color;
    // specular
    vec3 viewDir = normalize(camPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = 0.0;
    vec3 halfwayDir = normalize(lightDir + viewDir);
    spec = pow(max(dot(normal, halfwayDir), 0.0), 16.0);
    vec3 specular = vec3(0.3) * spec; // assuming bright white light color
    FragColor = vec4(ambient + diffuse + specular, 1.0);
}
