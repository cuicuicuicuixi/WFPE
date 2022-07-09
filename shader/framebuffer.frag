#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform sampler2D bloomBlur;

uniform bool hdr;
uniform bool gray;
uniform float exposure;
uniform float gamma;
uniform bool bloom;

void main()
{
    vec3 col = texture(screenTexture, TexCoords).rgb;
    vec3 bloomColor = texture(bloomBlur, TexCoords).rgb;
    if(hdr)
    {
        if(bloom)
            col += bloomColor;
        // reinhard
        // vec3 result = hdrColor / (hdrColor + vec3(1.0));
        // exposure
        vec3 result = vec3(1.0) - exp(-col * exposure);
        // also gamma correct while we're at it
        result = pow(result, vec3(1.0 / gamma));
        if(gray)
        {
            float a = 0.2126 * result.r + 0.7152 * result.g + 0.0722 * result.b;
            FragColor = vec4(a, a, a, 1.0);
        }
        else
        {
            FragColor = vec4(result, 1.0);
        }
    }
    else
    {
        vec3 result = pow(col, vec3(1.0 / gamma));
        if(gray)
        {
            float a = 0.2126 * result.r + 0.7152 * result.g + 0.0722 * result.b;
            FragColor = vec4(a, a, a, 1.0);
        }
        else
        {
            FragColor = vec4(result, 1.0);
        }
    }
}
