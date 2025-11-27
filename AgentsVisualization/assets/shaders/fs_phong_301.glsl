#version 300 es
precision highp float;

in vec3 v_normal;
in vec3 v_surfaceToLight;
in vec3 v_surfaceToView;
in vec4 v_color; // Receive the object color from the vertex shader

// Scene uniforms
uniform vec4 u_ambientLight;
uniform vec4 u_diffuseLight;
uniform vec4 u_specularLight;

// Model uniforms
uniform vec4 u_ambientColor;
uniform vec4 u_diffuseColor;
uniform vec4 u_specularColor;
uniform float u_shininess;

out vec4 outColor;

void main() {
    // v_normal must be normalized because the shader will interpolate
    // it for each fragment
    vec3 normal = normalize(v_normal);

    // Normalize the other incoming vectors
    vec3 surfToLightDirection = normalize(v_surfaceToLight);
    vec3 surfToViewDirection = normalize(v_surfaceToView);

    // CALCULATIONS FOR THE AMBIENT, DIFFUSE and SPECULAR COMPONENTS
    float diffuse = max(dot(normal, surfToLightDirection), 0.0);
    
    float specular = 0.0;
    if (diffuse > 0.0) {
        vec3 reflected = 2.0 * dot(surfToLightDirection, normal) * normal - surfToLightDirection;
        specular = pow(max(dot(reflected, surfToViewDirection), 0.0), u_shininess);
    }

    // Base color of the object
    vec4 baseColor = v_color * u_diffuseColor;

    // Compute the three parts of the Phong lighting model;
    vec4 ambientColor = u_ambientLight * u_ambientColor * baseColor;
    vec4 diffuseColor = u_diffuseLight * u_diffuseColor * diffuse * baseColor;
    vec4 specularColor = u_specularLight * u_specularColor * specular * baseColor;

    // Use the color of the texture on the object
    outColor = ambientColor + diffuseColor + specularColor;
}
