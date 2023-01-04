#version 330 core

in vec3 normal;
in vec2 texCoords;

out vec4 FragColor;

uniform sampler2D ourTexture;

void main()
{
	vec3 ambientLightColor = vec3(0.5,0.5,0.5);
	float ambientStrength = 0.3;
	vec3 ambient = ambientStrength * ambientLightColor;

	vec3 diffuseLightColor = vec3(0.5,0.5,0.5);
	vec3 lightDir = normalize(vec3(-0.4,-0.6,-0.2));
	vec3 diffuse = max(-dot(normal, lightDir),0.0) * diffuseLightColor;

	vec3 objectColor = texture(ourTexture, texCoords).xyz;
	vec3 result = (ambient + diffuse) * objectColor;

	FragColor = vec4(result, 1.0);
}