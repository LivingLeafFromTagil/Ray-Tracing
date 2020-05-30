#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"

//ширина окна
const int W_WIDTH = 1024;
//высота окна
const int W_HEIGHT = 768;
//угол обзора
const float FOV = M_PI / 3;

//свет
struct Light {
	Light(const Vec3f& p, const float i) : lightPosition(p), lightIntensity(i) {}
	//позиция источника
	Vec3f lightPosition;
	//интенсивность
	float lightIntensity;
};

//материал
struct Material {
	Material(const float r, const Vec4f& a, const Vec3f& color, const float spec) : refractiveIndex(r), albedo(a),
		diffuseColor(color), specularExponent(spec) {}
	Material() : refractiveIndex(1), albedo(1, 0, 0, 0), diffuseColor(), specularExponent() {}
	//коэффицент преломления
	float refractiveIndex;
	//альбедо (показатель диффузного отражения)
	Vec4f albedo;
	//цвет
	Vec3f diffuseColor;
	//показатель блестящей поверхности
	float specularExponent;
};

//сфера
struct Sphere {
	Sphere(const Vec3f& c, const float r, const Material& m) : center(c), radius(r), material(m) {}
	//центр сферы
	Vec3f center;
	//радиус сферы
	float radius;
	//материал сферы
	Material material;	

	//проверка на пересечение сферы лучом
	bool isRayIntersect(const Vec3f& origin, const Vec3f& dir, float& t0) const {
		Vec3f L = center - origin;
		float tca = L * dir;
		float d2 = L * L - tca * tca;
		if (d2 > radius * radius)
			return false;
		float thc = sqrtf(radius * radius - d2);
		t0 = tca - thc;
		float t1 = tca + thc;
		if (t0 < 0) {
			t0 = t1;
			if (t0 < 0)
				return false;
		}
		return true;
	}
};

//отражение света
Vec3f reflect(const Vec3f& I, const Vec3f& N) {
	return I - N * 2.f * (I * N);
}

//преломление света по Снеллиусу
Vec3f refract(const Vec3f& I, const Vec3f& N, const float eta_t, const float eta_i = 1.0f) {
	float cosi = -std::max(-1.0f, std::min(1.0f, I * N));
	if (cosi < 0)
		return refract(I, -N, eta_i, eta_t); //смена сред
	float eta = eta_i / eta_t;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? Vec3f(1, 0, 0) : I * eta + N * (eta * cosi - sqrtf(k));
}

//проверка пересечения объектов сцены
bool isSceneIntersected(const Vec3f& origin, const Vec3f& dir, const std::vector<Sphere>& spheres,
	Vec3f& hit, Vec3f& N, Material& material) {
	float spheres_dist = std::numeric_limits<float>::max();
	for (size_t i = 0; i < spheres.size(); i++) {
		float dist_i;
		if (spheres[i].isRayIntersect(origin, dir, dist_i) && dist_i < spheres_dist) {
			spheres_dist = dist_i;
			hit = origin + dir * dist_i;
			N = (hit - spheres[i].center).normalize();
			material = spheres[i].material;
		}
	}

	//вертикальная поверхность
	float boardVert = std::numeric_limits<float>::max();
	if (fabs(dir.z) > 1e-3) {
	    float dZ = -(origin.z + 26) / dir.z; 
	    Vec3f point = origin + dir * dZ;
	    if (dZ > 0 && fabs(point.x) < 15 && fabs(point.y) < 15  && dZ < spheres_dist) {
	        boardVert = dZ;
	        hit = point;
	        N = Vec3f(0, 0, 1);
	        material.diffuseColor = (int(.5 * hit.x + 1000) + int(.5 * hit.y)) & 1 ? Vec3f(1.0, 1.0, 1.0) : Vec3f(0.0, 0.0, 0.0);
	    }
	}

	return std::min(spheres_dist, boardVert) < 1000;
}

//создание луча
Vec3f castRay(const Vec3f& orig, const Vec3f& dir, const std::vector<Sphere>& spheres,
	const std::vector<Light>& lights, size_t depth = 0) {
	Vec3f point;
	Vec3f N;
	Material material;

	if (depth > 4 || !isSceneIntersected(orig, dir, spheres, point, N, material)) {
		return Vec3f(0.0, 1.0, 0.5); //цвет фона
	}

	Vec3f reflectionDir = reflect(dir, N);
	Vec3f reflectionOrigin = reflectionDir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
	Vec3f reflectionColor = castRay(reflectionOrigin, reflectionDir, spheres, lights, depth + 1);
	Vec3f refractionDir = refract(dir, N, material.refractiveIndex).normalize();
	Vec3f refractionOrigin = refractionDir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
	Vec3f refractionColor = castRay(refractionOrigin, refractionDir, spheres, lights, depth + 1);

	float diffuseLightIntensity = 0;
	float specularLightIntensity = 0;
	for (size_t i = 0; i < lights.size(); i++) {
		Vec3f lightDir = (lights[i].lightPosition - point).normalize();
		float lightDistance = (lights[i].lightPosition - point).norm();

		Vec3f shadowOrigin = lightDir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // checking if the point lies in the shadow of the lights[i]
		Vec3f shadowPoint;
		Vec3f shadowN;
		Material tmpMaterial;
		if (isSceneIntersected(shadowOrigin, lightDir, spheres, shadowPoint, shadowN, tmpMaterial)
			&& (shadowPoint - shadowOrigin).norm() < lightDistance)
			continue;

		diffuseLightIntensity += lights[i].lightIntensity * std::max(0.0f, lightDir * N);
		specularLightIntensity += powf(std::max(0.0f, reflect(lightDir, N) * dir), material.specularExponent) * lights[i].lightIntensity;
	}
	return material.diffuseColor * diffuseLightIntensity * material.albedo[0]
		+ Vec3f(1.0, 1.0, 1.0) * specularLightIntensity * material.albedo[1]
		+ reflectionColor * material.albedo[2] + refractionColor * material.albedo[3];
}

//отрисовка сцены
void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
	std::vector<Vec3f> frameBuffer(W_WIDTH * W_HEIGHT);

#pragma omp parallel for
	for (size_t j = 0; j < W_HEIGHT; j++) { // actual rendering loop
		for (size_t i = 0; i < W_WIDTH; i++) {
			float dir_x = (i + 0.5) - W_WIDTH / 2;
			float dir_y = -(j + 0.5) + W_HEIGHT / 2;    // this flips the image at the same time
			float dir_z = -W_HEIGHT / (2. * tan(FOV / 2));
			frameBuffer[i + j * W_WIDTH] = castRay(Vec3f(0, 0, 0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
		}
	}

	std::ofstream fBufferFile; 
	fBufferFile.open("./out.ppm", std::ios::binary);
	fBufferFile << "P6\n" << W_WIDTH << " " << W_HEIGHT << "\n255\n";
	for (size_t i = 0; i < W_HEIGHT * W_WIDTH; ++i) {
		Vec3f& c = frameBuffer[i];
		float max = std::max(c[0], std::max(c[1], c[2]));
		if (max > 1) c = c * (1.0 / max);
		for (size_t j = 0; j < 3; j++) {
			fBufferFile << (char)(255 * std::max(0.0f, std::min(1.0f, frameBuffer[i][j])));
		}
	}
	fBufferFile.close();
}

int main() {
	Material ivory(1.0, Vec4f(0.6, 0.3, 0.0, 0.0), Vec3f(0.4, 0.4, 0.3), 50);
	Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125);
	Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10);
	Material blue_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.0, 0.0, 0.3), 10);
	Material mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425);
	Material metal(1.0, Vec4f(0.7, 8.0, 0.1, 0.0), Vec3f(0.3, 0.3, 0.3), 600);

	std::vector<Sphere> spheres;
	spheres.push_back(Sphere(Vec3f(5, -5, -14), 2, ivory));
	spheres.push_back(Sphere(Vec3f(-7, 4, -17), 2, glass));
	spheres.push_back(Sphere(Vec3f(-4, 1, -20), 3, red_rubber));
	spheres.push_back(Sphere(Vec3f(-8, -5, -14), 1, red_rubber));
	spheres.push_back(Sphere(Vec3f(-1, -4, -14), 3, mirror));
	spheres.push_back(Sphere(Vec3f(7, 2, -14), 2, mirror));
	spheres.push_back(Sphere(Vec3f(4, -2, -16), 2, metal));
	spheres.push_back(Sphere(Vec3f(4, 6, -20), 2, blue_rubber));

	std::vector<Light> lights;
	lights.push_back(Light(Vec3f(-20, 0, 20), 1.5));
	lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
	lights.push_back(Light(Vec3f(30, -20, 30), 1.7));

	render(spheres, lights);

	return 0;
}