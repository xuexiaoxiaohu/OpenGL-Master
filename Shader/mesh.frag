#version 450 core

struct material{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 

struct dirLight{
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 vPosition;
in vec3 vNormal;  

uniform vec3 viewPos;
uniform material mtrl;
uniform dirLight dirLight1, dirLight2;

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir);

void main(){
    vec3 norm = normalize(vNormal);
    vec3 viewDir = normalize(viewPos - vPosition);

    vec3 result = calcDirLight(dirLight1, norm, viewDir);
    result += calcDirLight(dirLight2, norm, viewDir);

    gl_FragColor = vec4(result, 1.0);
} 

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir){
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), mtrl.shininess);
    vec3 ambient  = light.ambient * mtrl.ambient;
    vec3 diffuse  = light.diffuse * (diff * mtrl.diffuse);
    vec3 specular = light.specular * (spec * mtrl.specular); 
    return (ambient + diffuse + specular);
}