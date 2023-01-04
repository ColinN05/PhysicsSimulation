#version 330 core

layout (location = 0) in vec3 pos;
layout (location = 1) in vec2 in_texCoords;
layout (location = 2) in vec3 in_normal;

out vec3 normal;
out vec2 texCoords;

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

void main()
{
	gl_Position = proj * view * model * vec4(pos,1.0);

	normal = normalize(vec3(transpose(inverse(model)) * vec4(in_normal,1.0)));
	texCoords = in_texCoords;
}