#include "parser.h"
#include "utiliy.h"

using namespace parser;

struct Ray {
    Vec3f origin;
    
    // Normalized
    Vec3f direction;
};

struct Intersection {
    float t;
    int material_id;
    Vec3f intersectionPoint;

    // Normalized
    Vec3f normal;
};

Ray makeRay(const Camera &camera, int i, int j){
    float left = camera.near_plane.x;
    float right = camera.near_plane.y;
    float bottom = camera.near_plane.z;
    float top = camera.near_plane.w;

    // camera.gaze = -w
    // camera.up = v
    // m = e + (gaze * distance)
    Vec3f m; 
    m.x = camera.position.x + camera.gaze.x * camera.near_distance;
    m.y = camera.position.y + camera.gaze.y * camera.near_distance;
    m.z = camera.position.z + camera.gaze.z * camera.near_distance;
    
    // u = gaze X up
    Vec3f u;
    u = crossProduct(camera.gaze, camera.up);

    // q = m + left * u + top * v
    Vec3f q;
    q.x = m.x + left * u.x + top * camera.up.x;
    q.y = m.y + left * u.y + top * camera.up.y;
    q.z = m.z + left * u.z + top * camera.up.z;

    // su = (i + 0.5)(r - l) / width
    float su = (i + 0.5) * (right - left) / camera.image_width;
    // sv = (j + 0.5)(t - b) / height
    float sv = (j + 0.5) * (top - bottom) / camera.image_height;
    // s = q + su * u - sv * v
    Vec3f s;
    s.x = q.x + su * u.x - sv * camera.up.x;
    s.y = q.y + su * u.y - sv * camera.up.y;
    s.z = q.z + su * u.z - sv * camera.up.z;

    // ray r = e + (s-e)t = e + dt
    Ray ray;
    ray.origin = camera.position;
    ray.direction = normalize(substract(s, camera.position));

    return ray;
    
}

void getSphereIntersection(const Ray &ray ,const std::vector<Sphere> &spheres, const Scene &scene, std::vector<Intersection> &intersections) {
    for(const Sphere &sphere : spheres) {
        // p = point at surface
        // (p-c)(p-c) - R*R = 0 => Intersection
        // p = o + td (ray vector where t independent)
        // aim : find t

        // (o + td - c)(o + td -c) - R * R = 0
        
        // center of sphere
        Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];

        // calculate discriminant
        float Bsquare = (2 * dotProduct(ray.direction, substract(ray.origin, center))) * (2 * dotProduct(ray.direction, substract(ray.origin, center)));
        float A = dotProduct(ray.direction, ray.direction);
        float C = dotProduct(substract(ray.origin, center),substract(ray.origin, center)) - sphere.radius * sphere.radius;
        float discriminant = Bsquare - 4 * A * C;

        // if discriminant < 0 => no intersection
        if(discriminant < 0) {
            continue;
        }

        float t1 = ( -2 * (dotProduct(ray.direction, substract(ray.origin, center))) - sqrt(discriminant) ) / (2 * dotProduct(ray.direction, ray.direction));
        float t2 = ( -2 * (dotProduct(ray.direction, substract(ray.origin, center))) + sqrt(discriminant) ) / (2 * dotProduct(ray.direction, ray.direction));

        // minimum of (t1, t2)
        float tmin = (t1 < t2) ? t1 : t2;

        Intersection intersection;
        // set intersection point with tmin
        intersection.intersectionPoint.x = ray.origin.x + tmin * ray.direction.x;
        intersection.intersectionPoint.y = ray.origin.y + tmin * ray.direction.y;
        intersection.intersectionPoint.z = ray.origin.z + tmin * ray.direction.z;

        // normal = (intersection - center)
        intersection.normal = normalize(substract(intersection.intersectionPoint, center));

        // t value of intersection
        intersection.t = tmin;

        // material_id for lightning
        intersection.material_id = sphere.material_id;

        // add intersection to sphere intersections of current ray
        intersections.push_back(intersection);

    }
}

void getTriangleIntersection(const Ray &ray ,const std::vector<Triangle> &triangles, const Scene &scene, std::vector<Intersection> &intersections) {
    for(const Triangle &triangle : triangles) {
        // corners of triangle (a,b,c)
        Vec3f a = scene.vertex_data[triangle.indices.v0_id-1];
        Vec3f b = scene.vertex_data[triangle.indices.v1_id-1];
        Vec3f c = scene.vertex_data[triangle.indices.v2_id-1];

        // o + td = a + beta * (b-a) + gamma * (c-a)
        // td + beta * (a-b) + gamma * (a-c) = a-o
        Vec3f ab = substract(a,b);
        Vec3f ac = substract(a,c);
        Vec3f ao = substract(a,ray.origin);

        // cramers rule
        float detA = det(ab,ac,ray.direction);
        if(detA == 0) {
            continue;
        }

        // 0 <= beta
        float beta = det(ao,ac,ray.direction) / detA;
        if(beta < 0) {
            continue;
        }

        // 0 <= gamma
        float gamma = det(ab,ao,ray.direction) / detA;
        if(gamma < 0) {
            continue;
        }
        // beta + gamma <= 1
        if(beta + gamma > 1) {
            continue;
        }

        // 0 < t
        float t = det(ab,ac,ao) / detA;
        if(t <= 0) {
            continue;
        }

        // set intersection details
        Intersection intersection;
        intersection.intersectionPoint.x = ray.origin.x + t * ray.direction.x;
        intersection.intersectionPoint.y = ray.origin.y + t * ray.direction.y;
        intersection.intersectionPoint.z = ray.origin.z + t * ray.direction.z;

        intersection.normal = normalize(crossProduct(ab,ac));

        intersection.t = t;
        intersection.material_id = triangle.material_id;

        // add intersection to sphere intersections of current ray
        intersections.push_back(intersection);
    }
}

void getMeshIntersection(const Ray &ray ,const std::vector<Mesh> &meshes, const Scene &scene, std::vector<Intersection> &intersections) {
    std::vector<Triangle> triangles;
    for(const Mesh &mesh : meshes){
        for(Face face : mesh.faces){
            triangles.push_back(Triangle{mesh.material_id, face});
        }
    }
    getTriangleIntersection(ray, triangles, scene, intersections);
}

Intersection getIntersection(const Scene &scene, const Ray &ray) {
    std::vector<Intersection> intersections;

    // Get intersections
    getSphereIntersection(ray, scene.spheres, scene, intersections);
    getTriangleIntersection(ray, scene.triangles, scene, intersections);
    getMeshIntersection(ray,scene.meshes, scene, intersections);

    // Get intersection with minimum t
    Intersection validIntersection;
    // -1 indicates no intersection yet
    validIntersection.t = -1;

    // TODO: refactor
    for(Intersection &intersection : intersections) {
        if(validIntersection.t == -1){
            if(intersection.t > 0){
                validIntersection.intersectionPoint = intersection.intersectionPoint;
                validIntersection.normal = intersection.normal;
                validIntersection.t = intersection.t;
                validIntersection.material_id = intersection.material_id;
            }
        }
        else {
            if(intersection.t > 0 && intersection.t < validIntersection.t){
                validIntersection.intersectionPoint = intersection.intersectionPoint;
                validIntersection.normal = intersection.normal;
                validIntersection.t = intersection.t;
                validIntersection.material_id = intersection.material_id;
            }
        }
    }

    return validIntersection;
}

Vec3f getAmbiant(const Ray &ray, const Intersection &intersection, const Scene &scene){
    Vec3f ambiant = {0,0,0};
    ambiant.x = scene.materials[intersection.material_id-1].ambient.x * scene.ambient_light.x;
    ambiant.y = scene.materials[intersection.material_id-1].ambient.y * scene.ambient_light.y;
    ambiant.z = scene.materials[intersection.material_id-1].ambient.z * scene.ambient_light.z;
    return ambiant;
}

Vec3f getDiffuseAndSpecular(const Ray &ray, const Intersection &intersection, const Scene &scene){
    Vec3f specular = {0,0,0};
    Vec3f diffuse = {0,0,0};
    for(PointLight pointLight : scene.point_lights){
        // Vector from intersection point to light
        Vec3f wi = substract(pointLight.position, intersection.intersectionPoint);

        // distance from light source to intersection point
        float lightDistance = dotProduct(wi,wi);

        // Create shadow ray with epsilon
        Ray secondaryRay;
        secondaryRay.origin.x = intersection.intersectionPoint.x + intersection.normal.x * scene.shadow_ray_epsilon;
        secondaryRay.origin.y = intersection.intersectionPoint.y + intersection.normal.y * scene.shadow_ray_epsilon;
        secondaryRay.origin.z = intersection.intersectionPoint.z + intersection.normal.z * scene.shadow_ray_epsilon;       
        secondaryRay.direction = normalize(wi);

        // get intersection for shadow ray
        Intersection shadowIntersection = getIntersection(scene, secondaryRay);

        // TODO: light at the middle case
        Vec3f lightToShadowIntersectionVector = normalize(substract(shadowIntersection.intersectionPoint, pointLight.position));

        // if shadowRay intersects and light not in the middle => shadow
        if(shadowIntersection.t > 0 && dotProduct(lightToShadowIntersectionVector,wi) <= 0) {
            continue;
        }
        
        // Irradiance I/r^2
        Vec3f irradiance;
        irradiance.x = pointLight.intensity.x / lightDistance;
        irradiance.y = pointLight.intensity.y / lightDistance;
        irradiance.z = pointLight.intensity.z / lightDistance;

        // cos(theta)
        float cosTheta = dotProduct(normalize(wi), normalize(intersection.normal));
        // max{wi.n, 0}
        cosTheta = (cosTheta > 0) ? cosTheta : 0;

        // diffuse shading
        diffuse.x += scene.materials[intersection.material_id-1].diffuse.x * cosTheta * irradiance.x;
        diffuse.y += scene.materials[intersection.material_id-1].diffuse.y * cosTheta * irradiance.y;
        diffuse.z += scene.materials[intersection.material_id-1].diffuse.z * cosTheta * irradiance.z;

        // half-vector h
        Vec3f h = substract(normalize(wi), normalize(ray.direction));
        // cos(alfa)
        float cosAlfa = dotProduct(normalize(intersection.normal), normalize(h));
        // max{h.n, 0}
        cosAlfa = (cosAlfa > 0) ? cosAlfa : 0;

        // specular shading
        specular.x += scene.materials[intersection.material_id-1].specular.x * pow(cosAlfa, scene.materials[intersection.material_id-1].phong_exponent) * irradiance.x;
        specular.y += scene.materials[intersection.material_id-1].specular.y * pow(cosAlfa, scene.materials[intersection.material_id-1].phong_exponent) * irradiance.y;
        specular.z += scene.materials[intersection.material_id-1].specular.z * pow(cosAlfa, scene.materials[intersection.material_id-1].phong_exponent) * irradiance.z;
 
    }
    return Vec3f{
        diffuse.x + specular.x,
        diffuse.y + specular.y,
        diffuse.z + specular.z
    };
}

Vec3f getMirror(const Ray &ray, const Intersection &intersection, const Scene &scene, int recursion_depth){
    if(recursion_depth > 0 && scene.materials[intersection.material_id-1].is_mirror){
        Ray wr;
        wr.direction.x = ray.direction.x + intersection.normal.x * -2 * dotProduct(ray.direction, intersection.normal);
        wr.direction.y = ray.direction.y + intersection.normal.y * -2 * dotProduct(ray.direction, intersection.normal);
        wr.direction.z = ray.direction.z + intersection.normal.z * -2 * dotProduct(ray.direction, intersection.normal);
        wr.direction = normalize(wr.direction);

        // add epsilon to prevent self-intersection
        wr.origin.x = intersection.intersectionPoint.x + intersection.normal.x * scene.shadow_ray_epsilon;
        wr.origin.y = intersection.intersectionPoint.y + intersection.normal.y * scene.shadow_ray_epsilon;
        wr.origin.z = intersection.intersectionPoint.z + intersection.normal.z * scene.shadow_ray_epsilon;

        // reflection intersection
        Intersection reflectionIntersection = getIntersection(scene, wr);
        
        // recursive ray-tracing valid => mirror color
        if(reflectionIntersection.t >= 0) {
            Vec3f mirroredColor = add(getMirror(wr, reflectionIntersection, scene, recursion_depth-1),
                                        add(getDiffuseAndSpecular(wr, reflectionIntersection, scene),getAmbiant(wr,reflectionIntersection,scene)));
            //Vec3f mirroredColor = getColor(wr, reflectionIntersection, scene, recursion_depth-1);
            mirroredColor.x = mirroredColor.x * scene.materials[intersection.material_id-1].mirror.x;
            mirroredColor.y = mirroredColor.y * scene.materials[intersection.material_id-1].mirror.y;
            mirroredColor.z = mirroredColor.z * scene.materials[intersection.material_id-1].mirror.z;
            //return add(mirroredColor,getMirror(wr, reflectionIntersection, scene, recursion_depth-1));
            return mirroredColor;
        }
    }
    return Vec3f{0,0,0};
}

Vec3f getColor(const Ray &ray, const Intersection &intersection, const Scene &scene, int recursion_depth) {
    Vec3f colorVector;

    // No valid intersection
    if(intersection.t == -1) {
        colorVector.x = scene.background_color.x;
        colorVector.y = scene.background_color.y;
        colorVector.z = scene.background_color.z;
    }

    // Valid intersection
    else{
        // Get shadings
        Vec3f ambiant = getAmbiant(ray, intersection, scene);
        Vec3f diffuseAndSpecular = getDiffuseAndSpecular(ray, intersection, scene);
        Vec3f mirror = getMirror(ray,intersection,scene,recursion_depth);

        // Round for [R,G,B]
        colorVector = add(ambiant, add(diffuseAndSpecular,mirror));
        colorVector.x = (colorVector.x > 255) ? 255 : round(colorVector.x);
        colorVector.y = (colorVector.y > 255) ? 255 : round(colorVector.y);
        colorVector.z = (colorVector.z > 255) ? 255 : round(colorVector.z);
        
    }

    return colorVector;
}