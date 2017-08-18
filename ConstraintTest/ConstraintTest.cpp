#include <SDL.h>
#include "Renderer.h"
#include "PhysWorld.h"
#include "Logger.h"
 
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

    Logger logger;

    PhysWorld physWorld(Vector3(0.f, -98.1f, 0.f));
    //physWorld.logger = &logger;

    RigidBodyDesc bodyDescs[2];
    RigidBodyDesc& simpleBodySphere = bodyDescs[0];
    simpleBodySphere.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    simpleBodySphere.linearDamping = 0.2f;

    RigidBodyDesc& compoundBody = bodyDescs[1];
    compoundBody.shapes.push_back(Box(Vector3(5.f), Transform(Vector3(0.f, 0.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    compoundBody.linearDamping = 0.2f;

    const int numBodies = 10;

    RigidBodyDesc kinematicBody;
    kinematicBody.invMass = 0.f;
    kinematicBody.invInertia = Vector3(0.f, 0.f, 0.f);
    kinematicBody.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));

    int kinBodyIdx = physWorld.createRigidActor(Transform(Vector3(0.f, 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), kinematicBody);

    for(int i=0; i<numBodies-1; ++i)
    {
        physWorld.createRigidActor(Transform(Vector3(20.f * (i+1), 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), bodyDescs[1]);
    }

    for(int i=0; i<numBodies-1; ++i)
    {
        physWorld.createConstraint(i, Transform::identity(), i+1, Transform(Vector3(-5.f, 0.f, 0.f), Quaternion::identity()));
    }

    RigidActor* kinActor = physWorld.getActor(kinBodyIdx);
    

    PhysWorldDebugger physWorldDebugger(physWorld, renderer);

    SDL_Event event;
    bool quit = false;
    float r = 0.f;
    bool bMouseDown = false;
    float offsetX = 0.f;
    float offsetY = 0.f;
    bool bSimulate = false;
    int frame = 0;
    bool bDumpLogs = true;

    while (!quit)
    {
        while(SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                quit = 1;
            }
            
            if(event.type == SDL_MOUSEBUTTONDOWN)
            {
                bMouseDown = true;
            }

            if (event.type == SDL_MOUSEBUTTONUP)
            {
                bMouseDown = false;
                bSimulate = true;
            }

            if (event.type == SDL_MOUSEMOTION)
            {
                offsetX += event.motion.xrel;
                offsetY += event.motion.yrel;
            }
        }

        const float deltaTime = 1/30.f;

        if(bSimulate || frame < 200 || true)
        {
            bSimulate = false;
            physWorld.simulate(deltaTime);
            frame++;
            logger.advance();
        }
        else if(bDumpLogs)
        {
            bDumpLogs = false;
            logger.dumpLogs();
        }
        
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
