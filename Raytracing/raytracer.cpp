#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "raytracerFunctionality.h"
#include <thread>
// chrono for speed test
#include <chrono>

void thread_function_with_range(const Scene &scene ,const Camera &camera, int j, int i, unsigned char* image, int pixelNumber, int range){
    for(int k = 0; k < range ; k++){
        Ray ray = makeRay(camera, j+k, i);
        Intersection intersection = getIntersection(scene, ray);
        Vec3f colorVector = getColor(ray, intersection, scene, scene.max_recursion_depth);

        image[pixelNumber] = colorVector.x;
        image[pixelNumber+1] = colorVector.y;
        image[pixelNumber+2] = colorVector.z;

        pixelNumber += 3;
    }
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    //change this later just for vscode
    scene.loadFromXml(argv[1]);
    //scene.loadFromXml("../test_scenes/inputs/killeroo.xml");

    // SPEED-TEST PURPOSE
    //auto start = std::chrono::high_resolution_clock::now();

    for (parser::Camera camera : scene.cameras) {      
        unsigned char* image = new unsigned char [camera.image_width * camera.image_height * 3];
        
        // multi-thread
        int threadCount = 8;
        std::thread threads[threadCount];
        std::thread vThreads[threadCount];
        std::thread v2Threads[threadCount];
        std::thread v3Threads[threadCount];
        int range = camera.image_width / threadCount;
        for(int i = 0; i < camera.image_height; i+=4){
            for(int t = 0; t < threadCount; t++){
                threads[t] = std::thread(thread_function_with_range, scene, camera, range * t, i, image, 3 * (i * camera.image_width + range * t), range);
                vThreads[t] = std::thread(thread_function_with_range, scene, camera, range * t, i+1, image, 3 * ((i+1) * camera.image_width + range * t), range);
                v2Threads[t] = std::thread(thread_function_with_range, scene, camera, range * t, i+2, image, 3 * ((i+2) * camera.image_width + range * t), range);
                v3Threads[t] = std::thread(thread_function_with_range, scene, camera, range * t, i+3, image, 3 * ((i+3) * camera.image_width + range * t), range);
                
            }
            for(int t = 0; t < threadCount; t++){
                threads[t].join();
                vThreads[t].join();
                v2Threads[t].join();
                v3Threads[t].join();
            }
        }
        write_ppm(camera.image_name.c_str(), image, camera.image_width, camera.image_height);
    }
    
    // SPEED-TEST PURPOSE
    /*auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time taken by function: "
         << duration.count() << " milliseconds" << std::endl;*/
}
