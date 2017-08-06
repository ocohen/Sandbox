#include <SDL.h>
#include "Renderer.h"
#include "PhysWorld.h"
 
int main(int argc, char *argv[])
{
    SDL_Window* displayWindow;
    SDL_Init(SDL_INIT_VIDEO);
    displayWindow = SDL_CreateWindow("ConstraintTest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    SDL_GL_CreateContext(displayWindow);
    
    Renderer renderer(800, 600);
    Vector3 cameraPosition(0.f, 0.f, 100.f);
    Vector3 up(0.f, 1.f, 0.f);
    Vector3 target(0.f, 0.f, 0.f);
    
    renderer.setCameraLookAt(cameraPosition, target, up);

    PhysWorld physWorld(Vector3(0.f, -98.1f, 0.f));
    RigidBodyDesc simpleBody;
    simpleBody.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    simpleBody.linearDamping = 0.2f;

    const int numBodies = 9;

    for(int i=-numBodies+1; i<0; ++i)
    {
        physWorld.createRigidBody(Transform(Vector3(i * 10.f, 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), simpleBody);
    }

    RigidBodyDesc kinematicBody;
    kinematicBody.invMass = 0.f;
    kinematicBody.invInertia = Vector3(0.f, 0.f, 0.f);
    kinematicBody.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));

    int kinBodyIdx = physWorld.createRigidBody(Transform(Vector3(0.f, 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), kinematicBody);
    RigidBody& kinBody = physWorld.getBody(kinBodyIdx);

    for(int i=0; i<numBodies-1; ++i)
    {
        physWorld.createConstraint(i, i+1);
    }

    PhysWorldDebugger physWorldDebugger(physWorld, renderer);

    SDL_Event event;
    bool quit = false;
    float r = 0.f;
    while (!quit)
    {
        while(SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                quit = 1;
            }
        }

        const float deltaTime = 1/60.f;

        physWorld.simulate(deltaTime);
        //kinBody.bodyToWorld.translation.y = 0.f;//sin(r) * 20.f;
        //kinBody.bodyToWorld.translation.x = 50.f + cos(r) * 20.f;
        //r += deltaTime;
                
        renderer.clear();
        physWorldDebugger.debugDraw();
        renderer.flush();

        SDL_GL_SwapWindow(displayWindow);
    }
    
    SDL_Quit();
    
    return 0;
}
