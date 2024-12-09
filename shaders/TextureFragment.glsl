#version 330 core

uniform sampler2D tex;
uniform bool useTexture;
// The vertex UV coordinates
in vec2 vertUV;

layout (location=0) out vec4 outColour;

void main ()
{
    if (useTexture) {
        outColour = texture(tex, vertUV);
    } else {
        outColour = vec4(0.3, 0.3, 0.3, 1.0); // RGB = 0.5 (grey), Alpha = 1.0
    }
}
