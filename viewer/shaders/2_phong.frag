#version 410

float PI = 3.14159265358979323846264338327950288419716939937510;

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

float F(float cosinus)
{    
     float c_i = 1;
     float c_i_2 = (pow(eta, 2) + pow(cosinus, 2) - 1);
     if (c_i_2 >= 0) {c_i = pow(c_i_2, 1/2);}
     float F_s = pow(length((cosinus - c_i)/(cosinus + c_i)), 2);
     float F_p = pow(length((pow(eta, 2) * cosinus - c_i)/(pow(eta, 2) * cosinus + c_i)), 2);
     return (F_s + F_p) / 2;
}

float tan2(float cosinus)
{
     return (1 - pow(cosinus, 2)) / pow(cosinus, 2);
}

float D(float alpha, float cosinus)
{
     float Qui = 1;
     if (cosinus < 0) {
          return 0;
     }
     return (Qui / PI * pow(cosinus, 4)) * (pow(alpha, 2) / pow((pow(alpha, 2) + tan2(cosinus)), 2));
}

float G1(float alpha, float cosinus)
{
     return 2 / (1 + pow(1 + pow(alpha, 2) * tan2(cosinus), 1/2));
}

void main( void )
{
     // This is the place where there's work to be done

     // Normalisation
     vec4 lightVectorNorm = normalize(lightVector);
     vec4 eyeVectorNorm = normalize(eyeVector);
     vec4 vertNormalNorm = normalize(vertNormal);


     float ka = 0.2;
     float kd = 0.2;
     float ks = 0.6;
     float I = lightIntensity;

     vec4 H = normalize((eyeVectorNorm - lightVectorNorm));
     float cos_d = dot(eyeVectorNorm, H);

     vec4 ambiantLight = ka * vertColor * I;
     vec4 diffuseLight = kd * vertColor * max(dot(vertNormalNorm, lightVectorNorm), 0) * I;

     float alpha = (200 - shininess) / 200;

     fragColor = ambiantLight + diffuseLight;

     if (blinnPhong) {
          fragColor = fragColor + ks * F(cos_d) * vertColor * pow(max(dot(vertNormalNorm, H), 0), shininess) * I;
     } else {
          float cos_i = dot(vertNormalNorm, - lightVectorNorm);
          float cos_h = dot(vertNormalNorm, H);
          float cos_o = dot(vertNormalNorm, eyeVectorNorm);
          float Cs = cos_i * ((F(cos_d) * D(alpha, cos_h) * G1(alpha, cos_i) * G1(alpha, cos_o)) / (4 * cos_i * cos_o));
          fragColor = fragColor + ks * Cs * vertColor * I;
     }
}
