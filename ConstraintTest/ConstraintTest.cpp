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
    physWorld.logger = &logger;
    logger.enabled = false;

    RigidBodyDesc bodyDescs[3];
    RigidBodyDesc& simpleBodySphere = bodyDescs[0];
    simpleBodySphere.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    simpleBodySphere.linearDamping = 0.5f;
    simpleBodySphere.finalize();

    RigidBodyDesc& compoundBody = bodyDescs[1];
    compoundBody.shapes.push_back(Box(Vector3(5.f), Transform(Vector3(0.f, 0.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    compoundBody.linearDamping = 0.5f;
    compoundBody.finalize();

    RigidBodyDesc& compoundBody2 = bodyDescs[2];
    compoundBody2.shapes.push_back(Box(Vector3(10.f), Transform(Vector3(0.f, 0.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    compoundBody2.linearDamping = 0.5f;
    compoundBody2.finalize();

    RigidBodyDesc rootBody;
    rootBody.shapes.push_back(Sphere(5.f, Transform(Vector3(0.f), Quaternion(0.f, 0.f, 0.f, 1.f))));
    rootBody.finalize();

    const float chainOffset = 50.f;
    
    for(int chainIdx = 0; chainIdx < 2; ++chainIdx)
    {
        const int numBodies = 10;


        int kinBodyIdx = physWorld.createRigidActor(Transform(Vector3(0.f + chainIdx * chainOffset, 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), rootBody);

        for (int i = 0; i < numBodies - 1; ++i)
        {
            physWorld.createRigidActor(Transform(Vector3(10.f * (i + 1) + chainIdx * chainOffset, 40.f, 0.f), Quaternion(0.f, 0.f, 0.f, 1.f)), bodyDescs[(i + 1) % 2]);
        }

        int idx = physWorld.createConstraint(-1, Transform(Vector3(chainIdx * chainOffset, 40.f, 0.f) , Quaternion(0.f, 0.f, 0.f, 1.f)), + numBodies * chainIdx, Transform::identity());

        for (int i = 0; i < numBodies - 1; ++i)
        {
            physWorld.createConstraint(i +  numBodies * chainIdx, Transform(Vector3(5.f, 0.f, 0.f), Quaternion::identity()), i + 1 + numBodies * chainIdx, Transform(Vector3(-5.f, 0.f, 0.f), Quaternion::identity()));
        }

        physWorld.fixedConstraints = true;

    }


    

    PhysWorldDebugger physWorldDebugger(physWorld, renderer);

    SDL_Event event;
    bool quit = false;
    float r = 0.f;
    bool bMouseDown = false;
    float offsetX = 0.f;
    float offsetY = 0.f;
    bool bSimulate = false;
    int frame = 0;
    bool bDumpLogs = false;;

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

        for(int i =0; i< 2; ++i)
        {
        
        const float deltaTime = 1/60.f;

        if(true || bSimulate /*|| frame < 200 || true*/)
        {
            bSimulate = false;
            physWorld.simulate(deltaTime);
            frame++;
            logger.advance();
        }
        /*else if(frame < 210)
        {
            logger.enabled = true;
            bSimulate = true;
            bDumpLogs = true;
        }
        else if(bDumpLogs)
        {
            bDumpLogs = false;
            logger.dumpLogs();
        }*/
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
