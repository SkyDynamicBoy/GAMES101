//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
	
	Intersection p = intersect(ray);
	if (!p.happened)
	{
		return this -> backgroundColor;
	}
	Vector3f point_color = p.m -> getEmission();
	Vector3f ref = ray.direction;

	Vector3f L_dir(0); 
	float lightpdf;
	Intersection i = Intersection();
	sampleLight(i, lightpdf);
	Vector3f L_ind = normalize(i.coords - p.coords);
	Ray L_ray = Ray(p.coords, L_ind);
	Intersection has_block = intersect(L_ray);
	if (has_block.happened && has_block.obj -> hasEmit()) 
	{
		float normsquare = std::pow((i.coords - p.coords).norm(), 2);
		L_dir += i.emit * p.m -> eval(ref, L_ind, p.normal) * dotProduct(L_ind, p.normal) * dotProduct(-L_ind, i.normal) / normsquare / lightpdf;
	}
	

	Vector3f L_indir(0);
	float ksi = get_random_float();
	if (ksi <= RussianRoulette) 
	{
		Vector3f ind = p.m -> sample(ref, p.normal);
		Ray trace_ray = Ray(p.coords, ind);
		Intersection q = intersect(trace_ray);
		if (q.happened && !q.obj -> hasEmit()) 
		{
			L_indir += castRay(trace_ray, 0) * p.m -> eval(ref, ind, p.normal) * dotProduct(ind, p.normal) / p.m -> pdf(ref, ind, p.normal) / RussianRoulette;
		}
	}
	point_color += L_dir + L_indir;
	return point_color;

}
