#include <SDL.h>
#include "Renderer.h"
#include "Shape.h"
#include "ShapeRenderer.h"
#include <vector>
#include "GJK.h"
#include <algorithm>
 
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
    Transform boxTM(Vector3(21.f, 3.f, 0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), PI_OVER_TWO * 0.5f));

    std::vector<Transform> tms;
    tms.push_back(sphereTM);
    tms.push_back(boxTM);

    Vector3 green(0.f, 1.f, 0.f);
    Vector3 red(1.f, 0.f, 0.f);

    bool debugEnabled = false;
    int debugUpToFrame = 0;
    bool renderShapes = true;
    float zoom = 0.f;
    int debugTarget = 2;

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

            if(event.type == SDL_MOUSEWHEEL)
            {
                zoom -= event.wheel.y;
            }

            if(event.type == SDL_KEYDOWN)
            {
                if(event.key.keysym.sym == SDLK_d)
                {
                    debugEnabled = !debugEnabled;
                }

                if (event.key.keysym.sym == SDLK_f)
                {
                    debugTarget += 1;
                }
                
                if (event.key.keysym.sym == SDLK_r)
                {
                    renderShapes = !renderShapes;
                }

                if (event.key.keysym.sym == SDLK_1)
                {
                    debugUpToFrame -= 1;
                    debugUpToFrame = debugUpToFrame < 0 ? 0 : debugUpToFrame;
                }

                if (event.key.keysym.sym == SDLK_2)
                {
                    debugUpToFrame += 1;
                }
            }
        }

        renderer.setCameraPosition(Vector3(0.f, 0.f, 150.f + zoom));
        renderer.clear();

        if(tms.size() > 0)
        {
            tms[0].translation += Vector3(offsetX, offsetY, 0.f);
            offsetX = offsetY = 0.f;
        }

        GJKDebugInfo debugInfo;
        debugInfo.numIterations = 10000;
        debugInfo.hullResolution = 16;
        int debugI = debugTarget % 2;
        int debugJ = 1 - debugI;

        static float r = 0.f;
        r += 0.001f;

        tms[1] = Transform(Vector3(0.f), Quaternion::fromAxisAndAngle(Vector3(0.f, 0.f, 1.f), r));

        for(int i=0; i< shapes.size(); ++i)
        {
            bool bOverlap = false;
            for(int j=0; j<shapes.size(); ++j)
            {
                if(j == i){ continue; } //skip self

                const bool useDebug = debugEnabled && debugI == i && debugJ == j;
                if(gjkOverlappingImp<true>(ShapeUnion(shapes[i]), tms[i], ShapeUnion(shapes[j]), tms[j], useDebug ? &debugInfo : nullptr, fabs(sinf(r*100.f)*2.f)))
                {
                    bOverlap = true;
                    break;
                }
            }
            if(renderShapes)
            {
                renderShape(shapes[i].asShape(), tms[i], renderer, bOverlap ? &green : nullptr);
            }
        }
        
        if(debugEnabled)
        {
            for(int i=0; i<debugInfo.hullVerts.size()-1; ++i)
            {
                renderer.drawLine(debugInfo.hullVerts[i], debugInfo.hullVerts[i+1]);
            }
            renderer.drawLine(debugInfo.hullVerts[debugInfo.hullVerts.size()-1], debugInfo.hullVerts[0]);

            int numFramesToDraw = min((int)debugInfo.perFrameInfo.size()-1, debugUpToFrame);
            //for(size_t frame=0; frame< numFramesToDraw; ++frame)
            int frame = numFramesToDraw;
            {
                const GJKDebugPerFrameInfo& info = debugInfo.perFrameInfo[frame];

                renderer.drawCross(Transform::identity(), 3.f);

                renderer.drawPoint(info.closestPt, &Red);

                switch(info.dimension)
                {
                    case 0: renderer.drawPoint(info.simplex[0], &green); break;
                    case 1: renderer.drawLine(info.simplex[0], info.simplex[1], &green); break;
                    case 2: renderer.drawTriangle(info.simplex[0], info.simplex[1], info.simplex[2], &green); break;
                    case 3:
                    {
                        renderer.drawTriangle(info.simplex[0], info.simplex[1], info.simplex[2], &green);
                        renderer.drawTriangle(info.simplex[0], info.simplex[1], info.simplex[3], &green);
                        renderer.drawTriangle(info.simplex[0], info.simplex[2], info.simplex[3], &green);
                        renderer.drawTriangle(info.simplex[1], info.simplex[2], info.simplex[3], &green);
                    }
                    default: break;
                }
                
            }
        }


        renderer.flush();

        SDL_GL_SwapWindow(displayWindow);
    }
    
    SDL_Quit();
    
    return 0;
}
