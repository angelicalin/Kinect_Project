#version 330

// Fragment shader

// Textures
uniform sampler2D diffuseRamp;
uniform sampler2D specularRamp;

uniform vec3 eye_world;

uniform vec4 lightPosition;

uniform vec3 ka;
uniform vec3 kd;
uniform vec3 ks;
uniform float f;

uniform vec3 ambientLightIntensity;
uniform vec3 diffuseLightIntensity;
uniform vec3 specularLightIntensity;


// These get passed in from the vertex shader and are interpolated (varying) properties
// change for each pixel across the triangle:
in vec4 interpSurfPosition;
in vec3 interpSurfNormal;
in vec2 interpTexCoord;
// This is an out variable for the final color we want to render this fragment.
out vec4 fragColor;


void main() {

    // Start with black and then add lighting to the final color as we calculate it
	//vec3 finalColor = vec3(0.0, 0.0, 0.0);

    // TODO: Calculate ambient, diffuse, and specular lighting for this pixel based on its position, normal, etc.

	
    vec3 surface_position = vec3 (interpSurfPosition.x,interpSurfPosition.y,interpSurfPosition.z);
    vec3 eye_position = normalize(vec3(eye_world.x,eye_world.y,eye_world.z) - surface_position);
    vec3 light_position = normalize(vec3(lightPosition.x,lightPosition.y,lightPosition.z) -surface_position);
    vec3 normal_vector = normalize(interpSurfNormal);
    vec3 h = normalize(light_position+eye_position);
    
    float ndotl = max(dot(normal_vector, light_position),0);

	vec3 d = texture(diffuseRamp, vec2(ndotl,0)).rgb;
    vec3 diffuse = kd * diffuseLightIntensity * d;

	vec3 s = texture(specularRamp, vec2(pow(dot(h, normal_vector),f),0)).rgb;
    vec3 specular = ks*specularLightIntensity * s;
    
    vec3 ambient = ka*ambientLightIntensity;

    
 	// Tell OpenGL to use the r,g,b compenents of finalColor for the color of this fragment (pixel).
    fragColor.rgb = specular+ambient+diffuse;

	// And, set the alpha component to 1.0 (completely opaque, no transparency).
	fragColor.a = 1.0;
}
