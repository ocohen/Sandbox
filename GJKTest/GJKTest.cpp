#include <SDL.h>
#include "Renderer.h"
 
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
        }
            
                
        renderer.clear();
        renderer.flush();

        SDL_GL_SwapWindow(displayWindow);
    }
    
    SDL_Quit();
    
    return 0;
}
