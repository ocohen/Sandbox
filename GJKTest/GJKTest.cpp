#include <SDL.h>
#include "Renderer.h"
#include "Shape.h"
#include "ShapeRenderer.h"
#include <vector>
#include "GJK.h"
 
int main(int argc, char *argv[])
{
    SDL_Window* displayWindow;
    SDL_Init(SDL_INIT_VIDEO);
    displayWindow = SDL_CreateWindow("ConstraintTest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    SDL_GL_CreateContext(displayWindow);
    
    Renderer renderer(800, 600);
    Vector3 cameraPosition(0.f, 0.f, 150.f);
    Vector3 up(0.f, 1.f, 0.f);
    Vector3 target(0.f, 0.f, 0.f);
    
    renderer.setCameraLookAt(cameraPosition, target, up);

    SDL_Event event;
    bool quit = false;
    bool bMouseDown = false;
    float offsetX = 0.f;
    float offsetY = 0.f;

    Sphere sphere(10.f, Transform::identity());
    Box box(Vector3(10.f, 20.f, 1.f), Transform::identity());

    std::vector<ShapeUnion> shapes;
    shapes.push_back(sphere);
    shapes.push_back(box);

    Transform sphereTM = Transform::identity();
    Transform boxTM(Vector3(21.f, 0.f, 0.f));

    std::vector<Transform> tms;
    tms.push_back(sphereTM);
    tms.push_back(boxTM);

    Vector3 green(0.f, 1.f, 0.f);

    while (!quit)
    {
        while(SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                quit = 1;
            }

            if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                bMouseDown = true;
            }

            if (event.type == SDL_MOUSEBUTTONUP)
            {
                bMouseDown = false;
            }

            if (event.type == SDL_MOUSEMOTION)
            {
                if(bMouseDown)
                {
                    offsetX += event.motion.xrel;
                    offsetY += -event.motion.yrel;
                }
            }
        }

        renderer.clear();

        if(tms.size() > 0)
        {
            tms[0].translation += Vector3(offsetX, offsetY, 0.f);
            offsetX = offsetY = 0.f;
        }

        for(int i=0; i< shapes.size(); ++i)
        {
            bool bOverlap = false;
            for(int j=0; j<shapes.size(); ++j)
            {
                if(j == i){ continue; } //skip self

                if(gjkOverlapping(ShapeUnion(shapes[i]), tms[i], ShapeUnion(shapes[j]), tms[j]))
                {
                    bOverlap = true;
                    break;
                }
            }
            renderShape(shapes[i].asShape(), tms[i], renderer, bOverlap ? &green : nullptr);
        }
                
        renderer.flush();

        SDL_GL_SwapWindow(displayWindow);
    }
    
    SDL_Quit();
    
    return 0;
}
