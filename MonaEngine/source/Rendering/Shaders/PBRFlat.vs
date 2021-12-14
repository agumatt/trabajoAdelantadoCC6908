#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout(location = 0) uniform mat4 mvpMatrix;
layout(location = 1) uniform mat4 modelMatrix;
layout(location = 2) uniform mat4 modelInverseTransposeMatrix;

//out vec3 normal;
out vec3 worldPos;
out vec3 normal;

void main()
{
	normal = normalize(mat3(modelInverseTransposeMatrix) * aNormal);
	worldPos = vec3(modelMatrix * vec4(aPos,1.0f));
	gl_Position = mvpMatrix * vec4(aPos,1.0f);

}