#version 410
#define M_PI 3.1415926535897932384626433832795

uniform float lightIntensity;
uniform bool blinnPhong;
uniform float shininess;
uniform float eta;
uniform sampler2D shadowMap;

in vec4 eyeVector;
in vec4 lightVector;
in vec4 vertColor;
in vec4 vertNormal;
in vec4 lightSpace;

out vec4 fragColor;

float alpha = (200 - shininess) / 200;

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

float F(float cos_theta)
{
     float sin_theta_carre = 1. - pow(cos_theta, 2.);
     if (pow(eta, 2) < sin_theta_carre)
     {
          return 1; // Reflexion totale
     }
     float c_i = pow(pow(eta, 2) - sin_theta_carre, 0.5);
     float F_s = pow(abs((cos_theta - c_i) / (cos_theta + c_i)), 2);
     float F_p = pow(abs((pow(eta, 2) * cos_theta - c_i) / (pow(eta, 2) * cos_theta + c_i)), 2);
     return (F_s + F_p) / 2;
}

void main( void )
{
     // This is the place where there's work to be done
     vec4 eyeVector_norm = normalize(eyeVector);
     vec4 lightVector_norm = normalize(lightVector);
     vec4 vertNormal_norm = normalize(vertNormal);

     // Lighting parameters
     float ka = 0.2;
     float kd = 0.2;
     float ks = 0.6;
     float I = lightIntensity;

     // Ambient lighting
     vec4 Ca = ka * vertColor * I;

     // Diffuse lighting
     vec4 Cd = kd * vertColor * max(dot(vertNormal_norm, lightVector_norm), 0) * I;

     // Specular lighting
     vec4 halfVec = normalize(eyeVector_norm - lightVector_norm);
     float cos_theta = dot(halfVec, eyeVector_norm);
     float F_theta = F(cos_theta);
     vec4 Cs;

     if(blinnPhong)
     {
          // Blinn-Phong model
          Cs = F_theta * I * vertColor * pow(max(dot(vertNormal_norm, halfVec), 0), shininess);
          if (cos_theta > 1 )
          {
               Cs = vec4(255,0,0,1);
          }
     }
     else
     {    
          // Cook-Torrance model
          float cos_theta_h = dot(halfVec, vertNormal_norm);
          float D = D(cos_theta_h);
          
          float cos_theta_i = dot(-lightVector_norm, vertNormal_norm);
          float G_i  = G(cos_theta_i);
          
          float cos_theta_o = dot(eyeVector_norm, vertNormal_norm);
          float G_o  = G(cos_theta_o);

          Cs = I * vertColor * F_theta * D * G_i * G_o / (4 * cos_theta_i * cos_theta_o) * I * vertColor * cos_theta_i;
     }

     fragColor = Ca + Cd + ks * Cs;

}
