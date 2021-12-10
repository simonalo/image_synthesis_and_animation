#version 410
#define M_PI 3.14159265358979323846

// IN variables
in vec4 vertColor;
in vec4 vertNormal;
in vec4 position;

// OUT variables
out vec4 fragColor;

// UNIFORM variables
uniform mat4 mat_inverse;
uniform mat4 persp_inverse;
uniform sampler2D envMap;

uniform bool transparent;
uniform float shininess;
uniform float eta;

uniform vec3 lightPosition;
uniform float lightIntensity;

// PROJECT variables
const int nb_rebound_max = 12;
const int nb_compo = 4;
vec3[nb_rebound_max * nb_compo] stack;

float alpha = (200 - shininess) / 200;
vec4 resultColor = vec4(0, 0, 0, 1);

// Usefull functions to compute light
float G(float cos_theta)
{
    float cos_theta_carre = pow(cos_theta, 2.);
    float tan_theta_carre = (1 - cos_theta_carre) / cos_theta_carre;
    return 2. / (1. + pow(1 + pow(alpha, 2) * tan_theta_carre, 0.5));
}

float D(float cos_theta)
{
    float xi = 1;

    if (cos_theta <= 0)
    {
        return 0;
    }

    float cos_theta_carre = pow(cos_theta, 2.);
    float tan_theta_carre = (1 - cos_theta_carre) / cos_theta_carre;
    
    return xi / (M_PI * pow(cos_theta, 4)) * pow(alpha, 2) / pow(pow(alpha, 2) + tan_theta_carre, 2);
}

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


// SPHERES DEFINITION
struct Sphere
{
    vec3 center;
    float radius;
    float reflectance;
    int id;
    vec3 color;
};

const int nb_spheres = 3;
uniform Sphere spheres[nb_spheres] = Sphere[3](
    Sphere(vec3(0, 0, 0), 1, 1., 0, vec3(1, 0, 0)),
    Sphere(vec3(0, 3, 4), 4, 1., 1, vec3(0, 1, 0)),
    Sphere(vec3(0, 0, -4), 0.5, 1., 2, vec3(0, 0, 1))
);


// CODE
vec4 getColorFromEnvironment(in vec3 direction)
{
    vec3 u = normalize(direction);
    float phi = acos(u.y);
    float theta = atan(u.z, u.x);

    return texture2D(envMap, vec2((theta+M_PI) / (2*M_PI), phi / M_PI));
}

bool raySphereIntersect(in vec3 start, in vec3 direction, out vec3 newPoint, out Sphere sphere_intersect, in int id) {
    // Check if the ray intersect a sphere and return true if it's the case. id != -1 if start is in a sphere.
    float min_dist = -1;

    for (int j = 0; j < 3; ++j)
    {
        Sphere current_sphere = spheres[j];
        if (current_sphere.id == id)
        {
            continue;
        }
        vec3 cp = start - current_sphere.center;
        float dist = length(cp);
        vec3 u = normalize(direction);
        float a = dot(u, u);
        float b = dot(u, cp);
        float c = dot(cp, cp) - current_sphere.radius * current_sphere.radius;

        float delta = pow(b, 2) - dot(cp, cp) + pow(current_sphere.radius, 2);

        if (delta > 0)
        {
            float lambda = -b - sqrt(delta);
            if (lambda <= 0.001)
            {
                lambda = -b + sqrt(delta);
                if (lambda > 0.001 && (min_dist == -1 || dist < min_dist))
                {
                    min_dist = dist;
                    sphere_intersect = current_sphere;
                    newPoint = start + lambda * u;
                }
            }
            else
            {
                if (min_dist == -1 || dist < min_dist)
                {
                    min_dist = dist;
                    sphere_intersect = current_sphere;
                    newPoint = start + lambda * u;
                }
            }
        }
    }

    if (min_dist != -1)
    {
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

    // Step 2: ray direction
    vec3 u = normalize((mat_inverse * worldPos).xyz);
    vec3 eye = (mat_inverse * vec4(0, 0, 0, 1)).xyz;
    
    // Step 3 : Compute reflected
    // Number of ray
    int nb_rays = 0;

    // Variables used to compute rays
    vec3 normal = vec3(0); 
    float current_eta = eta;
    vec3 reflected_ray;
    vec3 refracted_ray;
    float current_Fresnel = 1.0;

    vec3 pointIntersect = vec3(0);
    vec3 start = eye.xyz;
    int id = -1;

    // Compute stack
    // /!\ WE HAVE ONLY REFLECTED RAYS
    while (nb_rays < nb_rebound_max)
    {
        Sphere sphere_intersect;
        // Check if there is an intersection
        if (raySphereIntersect(start, u, pointIntersect, sphere_intersect, id))
        {
            id = sphere_intersect.id;
            normal = normalize(pointIntersect - sphere_intersect.center);
            // Compute reflected and refracted rays
            reflected_ray = reflect(u, normal);
            u = vec3(reflected_ray.xyz);

            // Save variables in stack
            stack[nb_compo * nb_rays] = pointIntersect.xyz;
            stack[nb_compo * nb_rays + 1] = normal;
            stack[nb_compo * nb_rays + 2] = reflected_ray;
            stack[nb_compo * nb_rays + 3] = vec3(sphere_intersect.id, 0, 0);

            start = vec3(pointIntersect.xyz);
            nb_rays++;
        }
        else 
        {
            resultColor += getColorFromEnvironment(u);
            break;
        }
    }

    // Compute color
    for (int i = nb_rays-1; i >= 0; --i)
    {
        vec3 current_intersection = stack[i * nb_compo]; // Point of intersection
        vec4 normal = normalize(vec4(stack[i * nb_compo + 1], 0)); // Normal to point of intersection
        vec4 reflected_ray = normalize(vec4(stack[i * nb_compo + 2], 0)); // Reflected ray from point of intersection
        int id = int(stack[i * nb_compo + 3].x); // Id of the sphere intersected

        vec4 u = normalize(vec4(lightPosition - current_intersection, 0)); // Light vector

        // Lighting parameters
        float ka = 0.01;
        float kd = 0.001;
        float ks = 0.01;
        float I = lightIntensity;

        // Ambient lighting
        vec4 Ca = ka * vertColor * I;

        vec3 pointIntersect = vec3(0);
        Sphere sphere_intersect;

        if (raySphereIntersect(current_intersection, u.xyz, pointIntersect, sphere_intersect, id))
        {
            // If the ray intersect a sphere between the current sphere and the light, we only return ambient light
            resultColor += Ca ;
            continue;
        }


        vec4 halfVec = normalize(reflected_ray + u);
        float cos_theta = dot(halfVec, reflected_ray);
        float fresnel_coeff = F(cos_theta, current_eta);

        // Diffuse lighting
        vec4 Cd = kd * vertColor * max(dot(normal, u), 0) * I;

        // Specular lighting
        float F_theta = fresnel_coeff;
        vec4 Cs = ks * F_theta * I * vertColor * pow(max(dot(normal, halfVec), 0), shininess);
        if (cos_theta > 1 )
        {
            Cs = vec4(255,0,0,1);
        }

        vec4 color_temp = Ca + Cd + Cs;

        resultColor += color_temp + resultColor * fresnel_coeff;
    }

    fragColor = resultColor;
}
