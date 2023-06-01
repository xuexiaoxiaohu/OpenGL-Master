#version 450 core
layout (location = 0) in vec3 localPosition;
layout (location = 1) in vec3 localNormal;

out vec3 vPosition;
out vec3 vNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main(){
	vPosition = vec3(model * vec4(localPosition, 1.0));
	vNormal = mat3(transpose(inverse(model))) * localNormal;
	gl_Position = proj * view * vec4(vPosition, 1.0f);
}
 