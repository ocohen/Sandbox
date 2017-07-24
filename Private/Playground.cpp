#include <SDL.h>
#include "Renderer.h"
 
int main(int argc, char *argv[])
{
    SDL_Window* displayWindow;
	SDL_Init(SDL_INIT_VIDEO);
	displayWindow = SDL_CreateWindow("Playground", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
	SDL_GL_CreateContext(displayWindow);
    
	Renderer renderer(800, 600);
	Vector3 cameraPosition(0.f, 0.f, 3.f);
	Vector3 up(0.f, 1.f, 0.f);
	Vector3 target(0.f, 0.f, 0.f);
	Vector3 green(0.f, 1.f, 0.f);
	Vector3 red(1.f, 0.f, 0.f);

	float r=0.f;

	Vector3 vertices[] = {	target + Vector3(-1.f,-1.f,0.f),
							target + Vector3(1.f,-1.f,0.f),
							target + Vector3(-1.f,1.f,0.f),
							target + Vector3(1.f,1.f,0.f)
						 };

	unsigned short indices[] = {0,1,2,2,3,1}; 
	
    SDL_Event event;
    bool quit = false;
    while (!quit)
    {
        while(SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                quit = 1;
            }
        }

		cameraPosition[0] = 3 * sin(r);
		cameraPosition[2] = 3 * cos(r);
		renderer.setCameraLookAt(cameraPosition, target, up);
		r += 0.01f;

		renderer.clear();
		renderer.drawMesh(vertices, indices, sizeof(indices) / (sizeof(short) * 3), &red);
		renderer.drawMesh(vertices, indices, sizeof(indices) / (sizeof(short) * 3), &green, false, 2);
		renderer.flush();

		SDL_GL_SwapWindow(displayWindow);
    }
    
    SDL_Quit();
    
    return 0;
}