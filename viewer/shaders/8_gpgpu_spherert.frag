#version 410
#define M_PI 3.14159265358979323846

uniform mat4 mat_inverse;
uniform mat4 persp_inverse;
uniform sampler2D envMap;

uniform bool transparent;
uniform float shininess;
uniform float eta;

uniform vec3 center;
uniform float radius;

in vec4 position;

out vec4 fragColor;


float F(float cos_theta, float current_eta)
{
    float sin_theta_carre = 1. - pow(cos_theta, 2.);
    if (pow(current_eta, 2) < sin_theta_carre)
    {
        return 1; // Reflexion totale
    }
    float c_i = pow(pow(current_eta, 2) - sin_theta_carre, 0.5);
    float F_s = pow(abs((cos_theta - c_i) / (cos_theta + c_i)), 2);
    float F_p = pow(abs((pow(current_eta, 2) * cos_theta - c_i) / (pow(current_eta, 2) * cos_theta + c_i)), 2);
    return (F_s + F_p) / 2;
}

vec4 getColorFromEnvironment(in vec3 direction)
{
    vec3 u = normalize(direction);
    float phi = acos(u.y);
    float theta = atan(u.z, u.x);

    return texture2D(envMap, vec2((theta+M_PI) / (2*M_PI), phi / M_PI));
}

bool raySphereIntersect(in vec3 start, in vec3 direction, out vec3 newPoint) {
    vec3 cp = start - center;
    float dist = length(cp);
    vec3 u = normalize(direction);
    float a = dot(u, u);
    float b = dot(u, cp);
    float c = dot(cp, cp) - radius * radius;

    float delta = pow(b, 2) - dot(cp, cp) + pow(radius, 2);

    if (delta > 0)
    {
        float lambda = -b - sqrt(delta);
        if (lambda <= 0.001)
        {
            lambda = -b + sqrt(delta);
            if (lambda <= 0.001 )
            {
                newPoint = start;
                return false;
            }
        }
        newPoint = start + lambda * u;
        return true;
    }

    newPoint = start;
    return false;
}

void main(void)
{
    // Step 1: I need pixel coordinates. Division by w?
    vec4 worldPos = position;
    worldPos.z = 1; // near clipping plane
    worldPos = persp_inverse * worldPos;
    worldPos /= worldPos.w;
    worldPos.w = 0;
    worldPos = normalize(worldPos);

    // Step 2: ray direction:
    vec3 u = normalize((mat_inverse * worldPos).xyz);
    vec3 eye = (mat_inverse * vec4(0, 0, 0, 1)).xyz;
    
    // Step 3 : Compute reflected and refracted
    // Number of ray
    int nb_rebound_max = 14;
    int nb_rays = 0;

    // Variables used to compute rays
    vec3 normal = vec3(0); 
    float current_eta = eta;
    vec3 reflected_ray;
    vec3 refracted_ray;
    float current_Fresnel = 1.0;
    vec4 resultColor = vec4(0, 0, 0, 1);

    vec3 pointIntersect = vec3(0);
    vec3 start = eye.xyz;

    while (nb_rays < nb_rebound_max)
    {
        // Check if there is an intersection
        if (raySphereIntersect(start, u, pointIntersect))
        {
            if (nb_rays != 0) 
            {
                normal = normalize(center - pointIntersect);
                // Change eta each time we change of context
                current_eta = 1/eta;
            } 
            else
            {
                normal = normalize(pointIntersect - center);
            }
            
            // Compute reflected and refracted rays
            reflected_ray = reflect(u, normal);
            refracted_ray = refract(u, normal, 1 / current_eta);

            // Compute intensity with fresnel coefficient
            float cos_theta = dot(normal, normalize(reflected_ray));
            float intensity = F(cos_theta, current_eta);

            if (nb_rays == 0) 
            {
                u = vec3(refracted_ray.xyz);
                if (!transparent) {
                    intensity = 1.0;
                }
                resultColor += intensity * getColorFromEnvironment(reflected_ray);
                current_Fresnel *= (1 - intensity);
            } 
            else
            {
                u = vec3(reflected_ray.xyz);
                resultColor += current_Fresnel * (1 - intensity) * getColorFromEnvironment(refracted_ray);
                current_Fresnel *= intensity;
            }


            start = vec3(pointIntersect.xyz);
            nb_rays++;
        }
        else 
        {
            // No intersection : don't need more computing
            resultColor += getColorFromEnvironment(u);
            break;
        }
    }

    fragColor = resultColor;
}
